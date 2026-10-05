"""Préparation partagée des images caméra pour l'entraînement et l'autopilot."""

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


def prepare_camera_image(frame: np.ndarray) -> np.ndarray:
    """Retourne l'image CAM_B telle quelle, sans masque ni dessin superposé."""
    return _gray_u8(frame).copy()


def prepare_stereo_images(left_frame: np.ndarray, right_frame: np.ndarray) -> np.ndarray:
    """Retourne les deux vues brutes empilées dans l'ordre gauche, droite."""
    left = prepare_camera_image(left_frame)
    right = prepare_camera_image(right_frame)
    if left.shape != right.shape:
        raise ValueError(f"Les deux vues doivent avoir la même taille : {left.shape} / {right.shape}.")
    return np.stack((left, right), axis=0)


def resize_for_model(image: np.ndarray) -> np.ndarray:
    """Normalise une image de dataset ou de caméra au format réseau 120×160."""
    gray = _gray_u8(image)
    return cv2.resize(gray, MODEL_IMAGE_SIZE, interpolation=cv2.INTER_AREA)


def resize_stereo_for_model(images: np.ndarray) -> np.ndarray:
    """Redimensionne les deux vues en gardant l'axe caméra gauche/droite."""
    stereo = np.asarray(images)
    if stereo.ndim != 3 or stereo.shape[0] != 2:
        raise ValueError("Les images stéréo doivent avoir la forme [2, hauteur, largeur].")
    return np.stack((resize_for_model(stereo[0]), resize_for_model(stereo[1])), axis=0)
