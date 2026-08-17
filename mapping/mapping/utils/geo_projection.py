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
    # Drone tilt at capture time (aviation/FRD sign: +roll = right side down,
    # +pitch = nose up), body-frame, only non-zero when
    # detection.tilt_compensation is enabled -- see capture_waypoint.py.
    # The camera is rigidly mounted (no gimbal), so it tilts with the
    # airframe; pixel_to_local() casts a ray through this tilt instead of
    # assuming a perfectly nadir camera when these are non-zero.
    roll_deg: float = 0.0
    pitch_deg: float = 0.0


def compute_gsd(altitude_m: float, hfov_deg: float, resolution: Tuple[int, int]) -> float:
    """Ground sample distance in meters/pixel (same for both axes, since
    both are derived from the same focal length and pixel pitch).

    Args:
        altitude_m: Camera height above the ground plane, in meters.
        hfov_deg: Horizontal field of view of the camera, in degrees.
        resolution: (width_px, height_px) of the photo the GSD is for.

    Returns:
        Ground sample distance in meters per pixel (same value for both axes).
    """
    footprint_x, _ = camera_footprint(altitude_m, hfov_deg, resolution)
    return footprint_x / resolution[0]


def _rotate_2d(x: float, y: float, angle_deg: float) -> Tuple[float, float]:
    """Rotate a body-frame vector (x, y) by angle_deg into the arena frame.

    Args:
        x: X component of the vector to rotate.
        y: Y component of the vector to rotate.
        angle_deg: Rotation angle, in degrees, counterclockwise.

    Returns:
        (x, y): the rotated vector, in the same units as the input.
    """
    angle = math.radians(angle_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a


def pixel_to_local(
    pixel_col: float,
    pixel_row: float,
    resolution: Tuple[int, int],
    gsd_m_per_px: float,
    camera_altitude_m: float,
    pose: CapturePose,
    camera_yaw_offset_deg: float = 0.0,
    mount_forward_m: float = 0.0,
    mount_right_m: float = 0.0,
) -> Tuple[float, float]:
    """Project an image pixel onto the ground plane, in local arena coordinates.

    Casts a ray from the camera through the pixel and intersects it with the
    flat ground plane, instead of just scaling by a flat per-pixel GSD --
    the latter implicitly assumes a perfectly nadir camera, so it drifts
    under any drone tilt (the camera has no gimbal, it's rigidly mounted,
    so it tilts with the airframe). With `pose.roll_deg == pose.pitch_deg
    == 0.0` (the default -- see `CapturePose`) this reduces to *exactly*
    the same ground point the old flat-GSD formula gave (verified in this
    module's `_demo()` self-check), so it's a strict generalization, not
    an approximation swapped in for the untilted case.

    Args:
        pixel_col: Column (x) of the pixel in the photo, in pixels.
        pixel_row: Row (y) of the pixel in the photo, in pixels.
        resolution: (width_px, height_px) of the photo.
        gsd_m_per_px: Ground sample distance, in meters per pixel (see
            `compute_gsd`), used to back out the camera's focal length.
        camera_altitude_m: Camera height above the ground plane, in meters,
            at the moment the photo was taken.
        pose: Drone pose (position, heading offset, tilt) at capture time,
            in the local arena frame.
        camera_yaw_offset_deg: Fixed camera mount rotation about the
            boresight, in degrees (`camera.mount.yaw_offset_deg` in
            config.yml).
        mount_forward_m: Camera mount offset forward of the drone's body
            center, in meters.
        mount_right_m: Camera mount offset to the right of the drone's body
            center, in meters.

    Returns:
        (local_x, local_y): the ground point the pixel projects to, in
        local arena coordinates (meters).
    """
    width_px, height_px = resolution

    along_x_px = pixel_col - width_px / 2
    along_y_px = height_px / 2 - pixel_row

    # Pinhole ray, in focal-length units (tan(angle) off the boresight,
    # exact for both axes -- not a small-angle approximation). focal_px is
    # backed out from gsd/altitude instead of taking hfov_deg as a separate
    # parameter: gsd = 2*altitude*tan(hfov/2)/width and
    # focal_px = (width/2)/tan(hfov/2) combine to focal_px = altitude/gsd.
    focal_px = camera_altitude_m / gsd_m_per_px
    x, y, z = along_x_px / focal_px, along_y_px / focal_px, 1.0

    # Body frame from here on is X=forward, Y=right, Z=down (aviation/FRD,
    # matching mount_forward_m/mount_right_m and pose.roll_deg/pitch_deg).
    # Each step below is a 2D rotation of the pair of axes it turns about,
    # composed body-to-world: sensor->body (about Z, boresight -- a fixed
    # mount rotation, so applied before tilt), then roll (about X), pitch
    # (about the new Y), and finally heading/yaw (about world-vertical Z).
    x, y = _rotate_2d(x, y, camera_yaw_offset_deg)
    y, z = _rotate_2d(y, z, pose.roll_deg)
    z, x = _rotate_2d(z, x, pose.pitch_deg)
    x, y = _rotate_2d(x, y, pose.heading_offset_deg)

    # Ray-ground intersection: camera_altitude_m down (z, positive) to the
    # ground plane, along a ray of slope (x, y, z) -- t is how many ray
    # "z=1 steps" that takes.
    t = camera_altitude_m / z

    # Camera's own ground position: drone position + mount offset (forward/
    # right of the drone's body center), rotated into the arena frame by
    # how much the drone has yawed since takeoff. Not tilted by roll/pitch
    # too -- the offset is small (cm-scale) enough that the extra tilt
    # correction on top of the correction above is not worth the
    # complexity it would add.
    mount_x, mount_y = _rotate_2d(mount_forward_m, mount_right_m, pose.heading_offset_deg)
    camera_x, camera_y = pose.local_x + mount_x, pose.local_y + mount_y

    return camera_x + t * x, camera_y + t * y


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
        """Fit the local-to-GPS affine transform from the 4 arena corners.

        Args:
            arena_size_x_m: Arena extent along the local X axis, in meters.
            arena_size_y_m: Arena extent along the local Y axis, in meters.
            vertices_gps: GPS coordinates of the 4 arena corners (A/B/C/D,
                see class docstring), from config.yml's arena.vertices_gps.
        """
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
        """Convert a local arena point to GPS coordinates using the fitted affine transform.

        Args:
            local_x: Local X coordinate, in meters, relative to the arena center.
            local_y: Local Y coordinate, in meters, relative to the arena center.

        Returns:
            (lat, lon): the corresponding GPS latitude and longitude.
        """
        v = np.array([local_x, local_y, 1.0])
        lat = float(self._lat_coeffs @ v)
        lon = float(self._lon_coeffs @ v)
        return lat, lon


def _demo() -> None:
    # ponytail self-check: pixel_to_local() with zero tilt must reduce to
    # exactly the pre-tilt-compensation flat-GSD formula (regression
    # safety for the ray-ground intersection rewrite).
    resolution = (640, 480)
    gsd = 0.01
    altitude = 3.0
    pose = CapturePose(local_x=0.0, local_y=0.0, altitude_m=altitude, heading_offset_deg=0.0)

    x, y = pixel_to_local(320, 240, resolution, gsd, altitude, pose)
    assert abs(x) < 1e-9 and abs(y) < 1e-9, (x, y)  # center pixel -> straight below

    x, y = pixel_to_local(420, 240, resolution, gsd, altitude, pose)
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9, (x, y)  # 100px * 0.01 gsd = 1.0m

    x, y = pixel_to_local(320, 140, resolution, gsd, altitude, pose)
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9, (x, y)  # 100px above center -> +1.0m along Y

    heading_pose = CapturePose(
        local_x=0.0, local_y=0.0, altitude_m=altitude, heading_offset_deg=90.0
    )
    x, y = pixel_to_local(420, 240, resolution, gsd, altitude, heading_pose)
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9, (x, y)  # heading rotates X-shift into Y

    print('geo_projection self-check OK')


if __name__ == '__main__':
    _demo()
