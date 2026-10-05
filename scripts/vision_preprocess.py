"""Prétraitement partagé des images stéréo pour l'entraînement et l'autopilot."""

from __future__ import annotations

import cv2
import numpy as np

MODEL_IMAGE_SIZE = (160, 120)  # (largeur, hauteur)


def _gray_u8(frame: np.ndarray) -> np.ndarray:
    image = np.asarray(frame)
    if image.ndim == 3:
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image.ndim != 2:
        raise ValueError(f"Image caméra attendue en niveaux de gris ou couleur, reçu {image.shape}.")
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    return image


def make_mask_stereo(left_frame: np.ndarray, right_frame: np.ndarray) -> np.ndarray:
    """Produit une image binaire des différences visibles entre les deux caméras.

    Les flux CAM_B et CAM_C de l'OAK-D Lite sont monochromes et calibrés en usine.
    Le masque par différence met en évidence les contours et objets avec parallaxe,
    tout en conservant le format monochrome utilisé par le modèle.
    """
    left = _gray_u8(left_frame)
    right = _gray_u8(right_frame)
    if left.shape != right.shape:
        raise ValueError(f"Les deux images stéréo doivent avoir la même taille : {left.shape} / {right.shape}.")

    difference = cv2.absdiff(left, right)
    blurred = cv2.GaussianBlur(difference, (5, 5), 0)
    _threshold, mask = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def resize_for_model(image: np.ndarray) -> np.ndarray:
    """Normalise une image de dataset ou de caméra au format réseau 120×160."""
    gray = _gray_u8(image)
    return cv2.resize(gray, MODEL_IMAGE_SIZE, interpolation=cv2.INTER_AREA)
