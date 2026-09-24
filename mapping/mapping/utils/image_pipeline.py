"""Image correction utilities: lens distortion, color correction, sharpness.

No ROS dependency: operates on plain numpy/OpenCV images so it is testable
in isolation and reusable outside the state machine.
"""

from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from nectar.vision.camera.calibration.calibration import CameraCalibration

from mapping.config import CalibrationConfig


def load_calibration(config: CalibrationConfig) -> Tuple[np.ndarray, np.ndarray]:
    """Load camera intrinsics, from the paths in config.yml if given,
    otherwise fall back to the nectar-sdk's saved calibration.

    Args:
        config: Calibration section of config.yml (camera matrix / distortion
            coefficient file paths, optional).

    Returns:
        (camera_matrix, dist_coeffs): the 3x3 camera intrinsics matrix and
        the lens distortion coefficients.
    """
    if config.camera_matrix_path and config.distortion_path:
        camera_matrix = np.loadtxt(config.camera_matrix_path, delimiter=',')
        distortion = np.loadtxt(config.distortion_path, delimiter=',')
        return camera_matrix, distortion

    return CameraCalibration.load_calibration()


def undistort(img: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> np.ndarray:
    """Remove lens distortion, keeping the full undistorted field of view.

    Args:
        img: BGR image as captured by the camera, distorted.
        camera_matrix: 3x3 camera intrinsics matrix.
        dist_coeffs: Lens distortion coefficients.

    Returns:
        The undistorted BGR image.
    """
    h, w = img.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha=0
    )
    return cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)


def white_balance_gray_world(img: np.ndarray) -> np.ndarray:
    """Gray-world white balance: scales each BGR channel so its mean
    matches the overall gray mean, correcting color casts from artificial
    lighting.

    Args:
        img: BGR image, uint8.

    Returns:
        The white-balanced BGR image, uint8.
    """
    img_float = img.astype(np.float32)
    channel_means = img_float.mean(axis=(0, 1))
    gray_mean = channel_means.mean()

    scales = gray_mean / np.clip(channel_means, 1e-6, None)
    balanced = img_float * scales
    return np.clip(balanced, 0, 255).astype(np.uint8)


def correct_gamma(img: np.ndarray, gamma: float) -> np.ndarray:
    """Apply gamma correction via a lookup table.

    Args:
        img: BGR (or grayscale) image, uint8.
        gamma: Gamma value; 1.0 leaves the image unchanged, <1.0 brightens
            midtones, >1.0 darkens them.

    Returns:
        The gamma-corrected image, uint8.
    """
    if gamma == 1.0:
        return img
    inv_gamma = 1.0 / gamma
    table = ((np.arange(256) / 255.0) ** inv_gamma * 255).astype(np.uint8)
    return cv2.LUT(img, table)


def correct_color(
    img: np.ndarray, gray_world_white_balance: bool = True, gamma: float = 1.0
) -> np.ndarray:
    """Apply the configured color-correction steps in sequence.

    Args:
        img: BGR image, uint8.
        gray_world_white_balance: Whether to apply gray-world white balance
            (see `white_balance_gray_world`).
        gamma: Gamma value passed to `correct_gamma`; 1.0 is a no-op.

    Returns:
        The color-corrected BGR image, uint8.
    """
    result = img
    if gray_world_white_balance:
        result = white_balance_gray_world(result)
    result = correct_gamma(result, gamma)
    return result


def sharpness_score(img: np.ndarray) -> float:
    """Variance of the Laplacian: higher means sharper/more in-focus.

    Args:
        img: BGR or grayscale image.

    Returns:
        Sharpness score (Laplacian variance); higher is sharper.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def pick_sharpest(images: Sequence[np.ndarray]) -> Tuple[Optional[np.ndarray], float]:
    """Return the sharpest image (and its score) from a list of candidates.

    Args:
        images: Candidate images (BGR or grayscale); entries may be None,
            which are skipped.

    Returns:
        (best_img, best_score): the sharpest image and its sharpness score,
        or (None, -1.0) if `images` had no non-None entries.
    """
    best_img: Optional[np.ndarray] = None
    best_score = -1.0
    for img in images:
        if img is None:
            continue
        score = sharpness_score(img)
        if score > best_score:
            best_score = score
            best_img = img
    return best_img, best_score
