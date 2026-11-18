#!/usr/bin/env python3
"""Train and export a ROS-compatible camera + LiDAR imitation model."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

PROJECT_DIR = Path(__file__).resolve().parents[1]
AUTONOMY_PACKAGE = (
    PROJECT_DIR / 'ros2/ros2_ws/src/robocar_autonomy'
)
sys.path.insert(0, str(AUTONOMY_PACKAGE))
from robocar_autonomy.bc_preprocessing import (  # noqa: E402
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    SCAN_BINS,
    image_to_model_input,
)
from ros_behavioral_cloning import RosBehavioralCloning  # noqa: E402


DEFAULT_CSV = PROJECT_DIR / 'ros2/dataset_behavioral/demonstrations.csv'
DEFAULT_MODEL = PROJECT_DIR / 'ros2/models/behavioral_cloning.onnx'


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, default=DEFAULT_CSV)
    parser.add_argument('--model-out', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--learning-rate', type=float, default=1e-3)
    parser.add_argument('--max-range-m', type=float, default=8.0)
    parser.add_argument('--max-speed-mps', type=float, default=0.5)
    parser.add_argument('--max-steering-angle-rad', type=float, default=0.34)
    parser.add_argument('--stop-speed-threshold-mps', type=float, default=0.025)
    parser.add_argument('--stop-confidence-threshold', type=float, default=0.70)
    parser.add_argument('--stop-release-threshold', type=float, default=0.35)
    parser.add_argument('--device', choices=('auto', 'cpu', 'cuda'), default='auto')
    parser.add_argument('--seed', type=int, default=31)
    return parser.parse_args()


def load_rows(csv_path: Path, max_range: float, max_speed: float, max_steering: float):
    required = {
        'image_path', 'scan_normalized', 'steering_angle_rad', 'speed_mps',
        'max_range_m',
    }
    rows = []
    with csv_path.open('r', newline='', encoding='utf-8-sig') as handle:
        reader = csv.DictReader(handle)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f'CSV columns missing: {sorted(missing)}')
        for line_number, row in enumerate(reader, start=2):
            image_path = Path((row.get('image_path') or '').strip())
            if not image_path.is_absolute():
                image_path = csv_path.parent / image_path
            try:
                scan = np.asarray(json.loads(row['scan_normalized']), dtype=np.float32)
                steering = float(row['steering_angle_rad'])
                speed = float(row['speed_mps'])
                row_max_range = float(row['max_range_m'])
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError(f'Invalid values on CSV line {line_number}: {exc}') from exc
            if scan.shape != (SCAN_BINS,):
                raise ValueError(
                    f'Line {line_number}: expected {SCAN_BINS} LiDAR bins, got {scan.size}.'
                )
            if not math.isclose(row_max_range, max_range, rel_tol=1e-6, abs_tol=1e-6):
                raise ValueError(
                    f'Line {line_number}: LiDAR max range {row_max_range} m does not match '
                    f'--max-range-m {max_range}.'
                )
            if not np.all(np.isfinite(scan)) or not math.isfinite(steering) or not math.isfinite(speed):
                raise ValueError(f'Non-finite label or scan on CSV line {line_number}.')
            if not image_path.is_file():
                raise ValueError(f'Image on CSV line {line_number} is missing: {image_path}')
            image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'Image on CSV line {line_number} cannot be decoded: {image_path}')
            rows.append((
                image_path,
                np.clip(scan, 0.0, 1.0),
                float(np.clip(steering / max_steering, -1.0, 1.0)),
                float(np.clip(speed / max_speed, -1.0, 1.0)),
                steering,
                speed,
            ))
    if len(rows) < 40:
        raise ValueError(f'Only {len(rows)} samples found; record at least 40 first.')
    return rows


class DemonstrationDataset(Dataset):
    def __init__(self, rows, indices, stop_speed_threshold):
        self.rows = rows
        self.indices = indices
        self.stop_speed_threshold = stop_speed_threshold

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, index):
        image_path, scan, steering_norm, speed_norm, _steering, speed = self.rows[
            self.indices[index]
        ]
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        image_input = image_to_model_input(image)
        return (
            torch.from_numpy(image_input),
            torch.from_numpy(scan.astype(np.float32)),
            torch.tensor([steering_norm, speed_norm], dtype=torch.float32),
            torch.tensor(float(abs(speed) <= self.stop_speed_threshold), dtype=torch.float32),
        )


def select_device(preference):
    if preference == 'cpu':
        return torch.device('cpu')
    if preference == 'cuda':
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA was requested but is unavailable.')
        return torch.device('cuda')
    return torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def evaluate(model, loader, device, criterion_stop, stop_threshold):
    model.eval()
    total_loss = 0.0
    total = 0
    stop_tp = stop_fp = stop_fn = 0
    with torch.no_grad():
        for images, scans, actions, stop_labels in loader:
            images, scans = images.to(device), scans.to(device)
            actions, stop_labels = actions.to(device), stop_labels.to(device)
            output = model(images, scans)
            action_loss = nn.functional.mse_loss(output[:, :2], actions)
            stop_loss = criterion_stop(output[:, 2], stop_labels)
            batch = images.shape[0]
            total_loss += float((action_loss + 0.5 * stop_loss).item()) * batch
            total += batch
            predicted_stop = torch.sigmoid(output[:, 2]) >= stop_threshold
            expected_stop = stop_labels >= 0.5
            stop_tp += int(torch.logical_and(predicted_stop, expected_stop).sum().item())
            stop_fp += int(torch.logical_and(predicted_stop, ~expected_stop).sum().item())
            stop_fn += int(torch.logical_and(~predicted_stop, expected_stop).sum().item())
    precision = stop_tp / max(stop_tp + stop_fp, 1)
    recall = stop_tp / max(stop_tp + stop_fn, 1)
    return total_loss / max(total, 1), precision, recall


def main():
    args = parse_args()
    if min(args.max_range_m, args.max_speed_mps, args.max_steering_angle_rad) <= 0:
        print('[ERROR] Ranges and output limits must be positive.')
        return 1
    if args.epochs <= 0 or args.batch_size <= 0:
        print('[ERROR] epochs and batch size must be positive.')
        return 1
    if not 0.0 < args.stop_release_threshold < args.stop_confidence_threshold < 1.0:
        print('[ERROR] Stop thresholds must satisfy 0 < release < stop < 1.')
        return 1
    try:
        device = select_device(args.device)
        rows = load_rows(
            args.csv, args.max_range_m, args.max_speed_mps,
            args.max_steering_angle_rad,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'[ERROR] {exc}')
        return 1

    stop_flags = np.asarray([
        abs(row[5]) <= args.stop_speed_threshold_mps for row in rows
    ], dtype=bool)
    if not stop_flags.any() or stop_flags.all():
        print('[ERROR] Dataset needs both moving demonstrations and explicit stop samples.')
        return 1

    split = int(len(rows) * 0.8)
    if split < 2 or len(rows) - split < 2:
        print('[ERROR] Dataset is too small for a chronological validation split.')
        return 1
    train_set = DemonstrationDataset(rows, list(range(split)), args.stop_speed_threshold_mps)
    validation_set = DemonstrationDataset(rows, list(range(split, len(rows))), args.stop_speed_threshold_mps)
    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True, num_workers=0)
    validation_loader = DataLoader(validation_set, batch_size=args.batch_size, shuffle=False, num_workers=0)

    positives = int(stop_flags[:split].sum())
    negatives = split - positives
    if positives == 0 or negatives == 0:
        print('[ERROR] Training portion needs both stop and moving samples.')
        return 1
    validation_stops = int(stop_flags[split:].sum())
    if validation_stops == 0 or validation_stops == len(stop_flags) - split:
        print('[ERROR] Validation portion needs both stop and moving samples.')
        return 1
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    positive_weight = torch.tensor(
        [min(max(negatives / max(positives, 1), 1.0), 10.0)], device=device
    )
    criterion_stop = nn.BCEWithLogitsLoss(pos_weight=positive_weight)
    model = RosBehavioralCloning(SCAN_BINS).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate, weight_decay=1e-5)

    best_state = None
    best_loss = math.inf
    stale_epochs = 0
    for epoch in range(args.epochs):
        model.train()
        for images, scans, actions, stop_labels in train_loader:
            images, scans = images.to(device), scans.to(device)
            actions, stop_labels = actions.to(device), stop_labels.to(device)
            output = model(images, scans)
            action_loss = nn.functional.mse_loss(output[:, :2], actions)
            stop_loss = criterion_stop(output[:, 2], stop_labels)
            loss = action_loss + 0.5 * stop_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()

        val_loss, precision, recall = evaluate(
            model, validation_loader, device, criterion_stop,
            args.stop_confidence_threshold,
        )
        print(
            f'Epoch {epoch + 1:03d}/{args.epochs} | '
            f'val_loss={val_loss:.5f} | stop_precision={precision:.3f} | '
            f'stop_recall={recall:.3f}'
        )
        if val_loss < best_loss:
            best_loss = val_loss
            best_state = {key: value.detach().cpu().clone()
                          for key, value in model.state_dict().items()}
            stale_epochs = 0
        else:
            stale_epochs += 1
            if stale_epochs >= 12:
                break

    if best_state is None:
        print('[ERROR] Training produced no model checkpoint.')
        return 1
    model.load_state_dict(best_state)
    model.eval()
    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    dummy_image = torch.zeros((1, 1, IMAGE_HEIGHT, IMAGE_WIDTH), dtype=torch.float32)
    dummy_scan = torch.ones((1, SCAN_BINS), dtype=torch.float32)
    try:
        torch.onnx.export(
            model,
            (dummy_image, dummy_scan),
            str(args.model_out),
            input_names=['image', 'scan'],
            output_names=['actions'],
            opset_version=12,
            do_constant_folding=True,
        )
    except Exception as exc:
        print(f'[ERROR] ONNX export failed: {exc}')
        print('Install the onnx package in the training environment, then retry.')
        return 1

    metadata = {
        'input_image_width': IMAGE_WIDTH,
        'input_image_height': IMAGE_HEIGHT,
        'scan_bins': SCAN_BINS,
        'max_range_m': args.max_range_m,
        'max_speed_mps': args.max_speed_mps,
        'max_steering_angle_rad': args.max_steering_angle_rad,
        'stop_speed_threshold_mps': args.stop_speed_threshold_mps,
        'stop_confidence_threshold': args.stop_confidence_threshold,
        'stop_release_threshold': args.stop_release_threshold,
        'training_samples': len(rows),
        'stop_samples': int(stop_flags.sum()),
        'validation_loss': best_loss,
    }
    sidecar = args.model_out.with_suffix(args.model_out.suffix + '.json')
    sidecar.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    print(f'[OK] ONNX model: {args.model_out}')
    print(f'[OK] Model metadata: {sidecar}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
