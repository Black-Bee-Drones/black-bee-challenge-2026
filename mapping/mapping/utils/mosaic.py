"""Optional orthomosaic builder, for debug/report visualization only.

Not part of the scoring pipeline: base detection runs directly on the
individual undistorted photos (see states/mission/detect_bases.py).

Because the drone pose at each capture is already known, this warps each
photo straight onto a shared top-down canvas using that known geometry --
no feature-based overlap matching (cv2.Stitcher) is needed.
"""

from typing import List, Tuple

import cv2
import numpy as np

from mapping.utils.geo_projection import CapturePose, compute_gsd, pixel_to_local


def _local_to_canvas(
    local_x: float,
    local_y: float,
    arena_size_x_m: float,
    arena_size_y_m: float,
    canvas_gsd_m_per_px: float,
) -> Tuple[float, float]:
    col = (local_x + arena_size_x_m / 2) / canvas_gsd_m_per_px
    row = (arena_size_y_m / 2 - local_y) / canvas_gsd_m_per_px
    return col, row


def build_mosaic(
    captures: List[Tuple[CapturePose, np.ndarray]],
    arena_size_x_m: float,
    arena_size_y_m: float,
    altitude_m: float,
    hfov_deg: float,
    resolution: Tuple[int, int],
    canvas_gsd_m_per_px: float = 0.02,
    camera_yaw_offset_deg: float = 0.0,
) -> np.ndarray:
    canvas_w = max(1, round(arena_size_x_m / canvas_gsd_m_per_px))
    canvas_h = max(1, round(arena_size_y_m / canvas_gsd_m_per_px))
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    src_gsd = compute_gsd(altitude_m, hfov_deg, resolution)
    width_px, height_px = resolution
    src_corners = np.float32([[0, 0], [width_px, 0], [width_px, height_px], [0, height_px]])

    for pose, img in captures:
        dst_corners = []
        for px, py in src_corners:
            local_x, local_y = pixel_to_local(
                px, py, resolution, src_gsd, pose, camera_yaw_offset_deg
            )
            dst_corners.append(
                _local_to_canvas(local_x, local_y, arena_size_x_m, arena_size_y_m, canvas_gsd_m_per_px)
            )

        homography, _ = cv2.findHomography(src_corners, np.float32(dst_corners))
        if homography is None:
            continue

        warped = cv2.warpPerspective(img, homography, (canvas_w, canvas_h))
        mask = warped.sum(axis=2) > 0
        canvas[mask] = warped[mask]

    return canvas
