"""Small camera + LiDAR network used by the ROS 2 imitation controller."""

from __future__ import annotations

import torch
from torch import nn


class RosBehavioralCloning(nn.Module):
    """Predict normalized steering, speed, and an explicit stop logit."""

    def __init__(self, scan_bins: int = 180):
        super().__init__()
        if scan_bins <= 0:
            raise ValueError("scan_bins must be positive")
        self.scan_bins = scan_bins
        self.image_encoder = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2),
            nn.Conv2d(8, 16, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 6)),
            nn.Flatten(),
            nn.Linear(16 * 4 * 6, 64),
            nn.ReLU(inplace=True),
        )
        self.scan_encoder = nn.Sequential(
            nn.Linear(scan_bins, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 32),
            nn.ReLU(inplace=True),
        )
        self.action_head = nn.Sequential(
            nn.Linear(96, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 3),
        )

    def forward(self, image: torch.Tensor, scan: torch.Tensor) -> torch.Tensor:
        if image.ndim != 4 or image.shape[1] != 1:
            raise ValueError("image must have shape [batch, 1, height, width]")
        if scan.ndim != 2 or scan.shape[1] != self.scan_bins:
            raise ValueError(f"scan must have shape [batch, {self.scan_bins}]")
        features = torch.cat((self.image_encoder(image), self.scan_encoder(scan)), dim=1)
        raw = self.action_head(features)
        # Columns: steering in [-1, 1], speed in [-1, 1], stop logit.
        return torch.cat((torch.tanh(raw[:, :2]), raw[:, 2:3]), dim=1)
