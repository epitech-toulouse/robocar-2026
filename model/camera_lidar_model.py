"""Réseau de clonage comportemental fusionnant l'historique caméra et LiDAR."""

from __future__ import annotations

import torch
from torch import nn


class CameraLidarBehavioralCloning(nn.Module):
    """Prédit la position servo à partir de trois images et scans successifs."""

    def __init__(self, ray_count: int = 180, history_scans: int = 3):
        super().__init__()
        if ray_count <= 0 or history_scans <= 0:
            raise ValueError("ray_count et history_scans doivent être positifs.")
        self.ray_count = ray_count
        self.history_scans = history_scans

        self.image_encoder = nn.Sequential(
            nn.Conv2d(history_scans * 2, 16, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 48, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 6)),
            nn.Flatten(),
            nn.Linear(48 * 4 * 6, 128),
            nn.ReLU(inplace=True),
        )
        self.lidar_encoder = nn.Sequential(
            nn.Linear(ray_count * history_scans, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, 96),
            nn.ReLU(inplace=True),
        )
        self.steering_head = nn.Sequential(
            nn.Linear(128 + 96, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.15),
            nn.Linear(128, 1),
            nn.Sigmoid(),
        )

    def forward(self, images: torch.Tensor, scans: torch.Tensor) -> torch.Tensor:
        if images.ndim != 5:
            raise ValueError("images doit avoir la forme [batch, history, 2, hauteur, largeur].")
        if images.shape[1] != self.history_scans or images.shape[2] != 2:
            raise ValueError(
                "Historique image incompatible : "
                f"{images.shape[1]} instants × {images.shape[2]} vues reçu, "
                f"{self.history_scans} × 2 attendu."
            )
        if scans.ndim != 2 or scans.shape[1] != self.ray_count * self.history_scans:
            raise ValueError(
                f"scans doit avoir la forme [batch, {self.ray_count * self.history_scans}]."
            )
        batch_size, history, cameras, height, width = images.shape
        image_features = self.image_encoder(images.reshape(batch_size, history * cameras, height, width))
        lidar_features = self.lidar_encoder(scans)
        return self.steering_head(torch.cat((image_features, lidar_features), dim=1)).squeeze(1)
