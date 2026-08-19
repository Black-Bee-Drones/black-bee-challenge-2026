"""Known-geometry image compositing: warps photos onto a shared top-down
canvas using their already-known capture pose, no feature-based overlap
matching (cv2.Stitcher) needed. Two uses:

- `build_mosaic()`: optional debug/report orthomosaic of the whole arena.
  Not part of the scoring pipeline -- base detection runs directly on the
  individual undistorted photos (see states/mission/detect_bases.py).
- `merge_base_crop()`: composites just the handful of overlapping photos
  of one base into a single crop showing it whole, for the rare case none
  of them has it fully in frame on its own (base sitting on the seam
  between two waypoints' footprints) -- see states/mission/detect_bases.py.
"""

from typing import List, Optional, Tuple

import cv2
import numpy as np

from mapping.utils.geo_projection import CapturePose, compute_gsd, pixel_to_local


def _local_to_canvas(
    local_x: float,
    local_y: float,
    center_x_m: float,
    center_y_m: float,
    canvas_size_x_m: float,
    canvas_size_y_m: float,
    canvas_gsd_m_per_px: float,
) -> Tuple[float, float]:
    """Project a local arena point onto a top-down canvas's pixel grid.

    Args:
        local_x: Local X coordinate, in meters.
        local_y: Local Y coordinate, in meters.
        center_x_m: Local X coordinate the canvas is centered on, in meters.
        center_y_m: Local Y coordinate the canvas is centered on, in meters.
        canvas_size_x_m: Canvas width, in meters.
        canvas_size_y_m: Canvas height, in meters.
        canvas_gsd_m_per_px: Canvas ground sample distance, in meters per pixel.

    Returns:
        (col, row): the corresponding canvas pixel coordinates.
    """
    col = (local_x - center_x_m + canvas_size_x_m / 2) / canvas_gsd_m_per_px
    row = (canvas_size_y_m / 2 - (local_y - center_y_m)) / canvas_gsd_m_per_px
    return col, row


def canvas_to_local(
    col: float,
    row: float,
    center_x_m: float,
    center_y_m: float,
    canvas_size_x_m: float,
    canvas_size_y_m: float,
    canvas_gsd_m_per_px: float,
) -> Tuple[float, float]:
    """Inverse of `_local_to_canvas`: a pixel in a canvas built by
    `merge_base_crop()`/`build_mosaic()` -> local arena coordinates. The
    canvas is already a top-down orthographic composite (built from
    `pixel_to_local()`-projected photos), so this is flat/direct -- no
    camera geometry (altitude, tilt) involved, unlike `pixel_to_local()`
    itself, which projects a raw camera photo.

    Args:
        col: Column (x) of the pixel in the canvas.
        row: Row (y) of the pixel in the canvas.
        center_x_m: Local X coordinate the canvas is centered on, in meters.
        center_y_m: Local Y coordinate the canvas is centered on, in meters.
        canvas_size_x_m: Canvas width, in meters.
        canvas_size_y_m: Canvas height, in meters.
        canvas_gsd_m_per_px: Canvas ground sample distance, in meters per pixel.

    Returns:
        (local_x, local_y): the corresponding local arena coordinates, in meters.
    """
    local_x = col * canvas_gsd_m_per_px - canvas_size_x_m / 2 + center_x_m
    local_y = canvas_size_y_m / 2 - row * canvas_gsd_m_per_px + center_y_m
    return local_x, local_y


def _warp_onto_canvas(
    captures: List[Tuple[CapturePose, np.ndarray, float, float]],
    canvas: np.ndarray,
    center_x_m: float,
    center_y_m: float,
    canvas_size_x_m: float,
    canvas_size_y_m: float,
    resolution: Tuple[int, int],
    canvas_gsd_m_per_px: float,
    camera_yaw_offset_deg: float = 0.0,
    mount_forward_m: float = 0.0,
    mount_right_m: float = 0.0,
) -> None:
    """Warp each (pose, img, gsd, camera_altitude_m) in `captures` onto
    `canvas` in-place, last-writer-wins where photos overlap (no
    blending -- fine for both callers here: a debug mosaic and a small
    base crop, neither scored on seam quality).

    Args:
        captures: (pose, img, gsd_m_per_px, camera_altitude_m) per photo to
            warp onto the canvas.
        canvas: Destination BGR canvas, modified in place.
        center_x_m: Local X coordinate the canvas is centered on, in meters.
        center_y_m: Local Y coordinate the canvas is centered on, in meters.
        canvas_size_x_m: Canvas width, in meters.
        canvas_size_y_m: Canvas height, in meters.
        resolution: (width_px, height_px) of the source photos.
        canvas_gsd_m_per_px: Canvas ground sample distance, in meters per pixel.
        camera_yaw_offset_deg: Fixed camera mount rotation about the
            boresight, in degrees.
        mount_forward_m: Camera mount offset forward of the drone's body
            center, in meters.
        mount_right_m: Camera mount offset to the right of the drone's body
            center, in meters.

    Returns:
        None. `canvas` is modified in place.
    """
    canvas_h, canvas_w = canvas.shape[:2]
    width_px, height_px = resolution
    src_corners = np.float32([[0, 0], [width_px, 0], [width_px, height_px], [0, height_px]])

    for pose, img, gsd, camera_altitude_m in captures:
        dst_corners = []
        for px, py in src_corners:
            local_x, local_y = pixel_to_local(
                px, py, resolution, gsd, camera_altitude_m, pose,
                camera_yaw_offset_deg=camera_yaw_offset_deg,
                mount_forward_m=mount_forward_m,
                mount_right_m=mount_right_m,
            )
            dst_corners.append(
                _local_to_canvas(
                    local_x, local_y, center_x_m, center_y_m,
                    canvas_size_x_m, canvas_size_y_m, canvas_gsd_m_per_px,
                )
            )

        homography, _ = cv2.findHomography(src_corners, np.float32(dst_corners))
        if homography is None:
            continue

        warped = cv2.warpPerspective(img, homography, (canvas_w, canvas_h))
        mask = warped.sum(axis=2) > 0
        canvas[mask] = warped[mask]


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
    """Build a debug/report top-down orthomosaic of the whole arena.

    Not part of the scoring pipeline -- base detection runs directly on the
    individual undistorted photos (see states/mission/detect_bases.py).

    Args:
        captures: (pose, img) per photo taken during the coverage flight.
        arena_size_x_m: Arena extent along the local X axis, in meters.
        arena_size_y_m: Arena extent along the local Y axis, in meters.
        altitude_m: Flight altitude the photos were taken at, in meters.
        hfov_deg: Horizontal field of view of the camera, in degrees.
        resolution: (width_px, height_px) of the source photos.
        canvas_gsd_m_per_px: Output canvas ground sample distance, in
            meters per pixel.
        camera_yaw_offset_deg: Fixed camera mount rotation about the
            boresight, in degrees.

    Returns:
        BGR canvas covering the whole arena, composited from `captures`.
    """
    canvas_w = max(1, round(arena_size_x_m / canvas_gsd_m_per_px))
    canvas_h = max(1, round(arena_size_y_m / canvas_gsd_m_per_px))
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    gsd = compute_gsd(altitude_m, hfov_deg, resolution)
    _warp_onto_canvas(
        [(pose, img, gsd, altitude_m) for pose, img in captures],
        canvas, 0.0, 0.0, arena_size_x_m, arena_size_y_m,
        resolution, canvas_gsd_m_per_px, camera_yaw_offset_deg,
    )
    return canvas


def merge_base_crop(
    captures: List[Tuple[CapturePose, np.ndarray, float, float]],
    base_local_x: float,
    base_local_y: float,
    base_size_m: float,
    resolution: Tuple[int, int],
    camera_yaw_offset_deg: float = 0.0,
    mount_forward_m: float = 0.0,
    mount_right_m: float = 0.0,
    margin_frac: float = 0.5,
) -> Optional[np.ndarray]:
    """Composite the photos in `captures` (pose, corrected img, gsd,
    camera_altitude_m -- one tuple per confirming photo of the same base,
    see BaseCandidate/BaseResult.cluster in utils/base_detector.py) into a
    single crop centered on the base, showing it whole even though no
    individual photo did. Same warp as build_mosaic(), just a small window
    around the base instead of the whole arena, and at the source photos'
    own (fine) GSD instead of the coarse debug-mosaic default.

    Returns None if there's nothing to composite (caller should keep
    whatever partial crop it already had as a fallback).

    Args:
        captures: (pose, corrected img, gsd_m_per_px, camera_altitude_m) per
            confirming photo of the same base.
        base_local_x: Local X coordinate of the base's estimated center, in meters.
        base_local_y: Local Y coordinate of the base's estimated center, in meters.
        base_size_m: Physical side length of the base square, in meters.
        resolution: (width_px, height_px) of the source photos.
        camera_yaw_offset_deg: Fixed camera mount rotation about the
            boresight, in degrees.
        mount_forward_m: Camera mount offset forward of the drone's body
            center, in meters.
        mount_right_m: Camera mount offset to the right of the drone's body
            center, in meters.
        margin_frac: Extra margin around the base, as a fraction of
            `base_size_m`, included in the output crop's window.

    Returns:
        BGR crop centered on the base, or None if `captures` was empty or
        nothing was actually warped onto the canvas.
    """
    if not captures:
        return None

    window_m = base_size_m * (1 + margin_frac)
    canvas_gsd = min(gsd for _, _, gsd, _ in captures)
    canvas_size = max(1, round(window_m / canvas_gsd))
    canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

    _warp_onto_canvas(
        captures, canvas, base_local_x, base_local_y, window_m, window_m,
        resolution, canvas_gsd, camera_yaw_offset_deg, mount_forward_m, mount_right_m,
    )
    return canvas if canvas.any() else None


def _demo() -> None:
    # ponytail self-check: two nadir photos, each missing one edge of the
    # base (it sits on the seam between them, like two overlapping
    # waypoints), merge into a crop that has both edges -- neither source
    # photo does on its own.
    resolution = (40, 40)
    gsd = 0.01
    altitude = 3.0
    base_size = 0.3

    def render(local_x: float) -> np.ndarray:
        img = np.zeros((40, 40, 3), dtype=np.uint8)
        col0 = round((-base_size / 2 - local_x) / gsd) + 20
        col1 = round((base_size / 2 - local_x) / gsd) + 20
        row0 = round(20 - (base_size / 2) / gsd)
        row1 = round(20 - (-base_size / 2) / gsd)
        img[max(0, row0):min(40, row1), max(0, col0):min(40, col1)] = 255
        return img

    pose_a = CapturePose(local_x=-0.08, local_y=0.0, altitude_m=altitude)
    pose_b = CapturePose(local_x=0.08, local_y=0.0, altitude_m=altitude)
    img_a, img_b = render(-0.08), render(0.08)

    merged = merge_base_crop(
        [(pose_a, img_a, gsd, altitude), (pose_b, img_b, gsd, altitude)],
        base_local_x=0.0, base_local_y=0.0, base_size_m=base_size,
        resolution=resolution,
    )
    assert merged is not None
    h, w = merged.shape[:2]
    assert merged[h // 2, 3].sum() == 0, 'left margin should be outside the base'
    assert merged[h // 2, w - 4].sum() == 0, 'right margin should be outside the base'
    assert merged[h // 2, w // 2].sum() > 0, 'center should be inside the base'

    # canvas_to_local() is the exact inverse of _local_to_canvas().
    window_m = base_size * 1.5
    col, row = _local_to_canvas(0.05, -0.02, 0.0, 0.0, window_m, window_m, gsd)
    back_x, back_y = canvas_to_local(col, row, 0.0, 0.0, window_m, window_m, gsd)
    assert abs(back_x - 0.05) < 1e-9 and abs(back_y - (-0.02)) < 1e-9

    print('mosaic self-check OK')


if __name__ == '__main__':
    _demo()
