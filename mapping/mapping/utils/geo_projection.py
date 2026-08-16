"""Pixel -> local arena coordinate -> GPS projection for a nadir camera.

No ROS dependency: pure geometry, testable in isolation.

Local frame convention
-----------------------
Origin at the arena center (== takeoff position, per the competition
rules). +X is "forward" and +Y is "right" relative to the drone's heading
at takeoff -- the same convention used for `drone.move_to(x, y,
reference=MoveReference.TAKEOFF)`. This is why `utils.coverage.Waypoint`
positions can be passed straight to `move_to` without any conversion.

`utils.coverage.compute_grid` lays its 2 columns out along X using the
camera's wide (horizontal-FOV, image-width) axis, and its rows along Y
using the narrow (vertical-FOV, image-height) axis -- so by default
(`camera_yaw_offset_deg=0`) the image's width axis is assumed mounted
along the drone's forward (X) axis. If the camera is actually mounted
rotated 90 deg relative to that, set `camera.mount.yaw_offset_deg` in
config.yml to correct it.
"""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from mapping.config import ArenaVertices
from mapping.utils.coverage import camera_footprint


@dataclass(frozen=True)
class CapturePose:
    """Drone pose at the moment a photo was taken, in the local arena frame."""

    local_x: float
    local_y: float
    altitude_m: float
    heading_offset_deg: float = 0.0  # current heading minus heading at takeoff


def compute_gsd(altitude_m: float, hfov_deg: float, resolution: Tuple[int, int]) -> float:
    """Ground sample distance in meters/pixel (same for both axes, since
    both are derived from the same focal length and pixel pitch).
    """
    footprint_x, _ = camera_footprint(altitude_m, hfov_deg, resolution)
    return footprint_x / resolution[0]


def pixel_to_local(
    pixel_col: float,
    pixel_row: float,
    resolution: Tuple[int, int],
    gsd_m_per_px: float,
    pose: CapturePose,
    camera_yaw_offset_deg: float = 0.0,
) -> Tuple[float, float]:
    """Project an image pixel onto the ground plane, in local arena coordinates."""
    width_px, height_px = resolution

    # Image width axis (wide/HFOV) -> arena X; image height axis (narrow/VFOV) -> arena Y,
    # matching the layout utils.coverage.compute_grid assumes (see module docstring).
    along_x_px = pixel_col - width_px / 2
    along_y_px = height_px / 2 - pixel_row  # image rows grow downward

    along_x_m = along_x_px * gsd_m_per_px
    along_y_m = along_y_px * gsd_m_per_px

    angle = math.radians(pose.heading_offset_deg + camera_yaw_offset_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    rotated_x = along_x_m * cos_a - along_y_m * sin_a
    rotated_y = along_x_m * sin_a + along_y_m * cos_a

    return pose.local_x + rotated_x, pose.local_y + rotated_y


class LocalToGpsTransform:
    """Affine transform from local arena meters to GPS lat/lon, fitted
    (least squares) from the 4 known arena corners.

    Corners are named A/B/C/D exactly like competition organizers call them
    out (front-left, front-right, back-left, back-right, relative to the
    drone's nose at takeoff -- not compass directions, since heading at
    takeoff isn't known in advance). See config.yml's arena.vertices_gps.
    """

    def __init__(
        self,
        arena_size_x_m: float,
        arena_size_y_m: float,
        vertices_gps: ArenaVertices,
    ):
        half_x, half_y = arena_size_x_m / 2, arena_size_y_m / 2
        # +X = forward, +Y = right (see module docstring).
        local_corners = [
            (half_x, -half_y),   # A: front-left
            (half_x, half_y),    # B: front-right
            (-half_x, -half_y),  # C: back-left
            (-half_x, half_y),   # D: back-right
        ]
        gps_corners = [vertices_gps.a, vertices_gps.b, vertices_gps.c, vertices_gps.d]

        design_matrix = np.array([[x, y, 1.0] for x, y in local_corners])
        lats = np.array([v.lat for v in gps_corners])
        lons = np.array([v.lon for v in gps_corners])

        self._lat_coeffs, *_ = np.linalg.lstsq(design_matrix, lats, rcond=None)
        self._lon_coeffs, *_ = np.linalg.lstsq(design_matrix, lons, rcond=None)

    def to_gps(self, local_x: float, local_y: float) -> Tuple[float, float]:
        v = np.array([local_x, local_y, 1.0])
        lat = float(self._lat_coeffs @ v)
        lon = float(self._lon_coeffs @ v)
        return lat, lon
