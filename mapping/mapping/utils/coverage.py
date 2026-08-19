"""Flight-grid planner for 100% camera coverage of a rectangular arena.

No ROS dependency: pure geometry, so it can be unit tested in isolation.

The camera footprint on the ground is a rectangle, not a square: with a
16:9 sensor and only the horizontal FOV known, the vertical FOV (and thus
the footprint's Y extent) must be derived from the sensor aspect ratio.
Using a square footprint (as if height == width) undercounts the number
of rows needed and leaves the arena's Y edges uncovered.

The grid assumes a constant drone yaw for the whole mission, with the
camera's wide (horizontal-FOV) axis aligned to the arena's X axis.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw_deg: float


def hfov_from_dfov(dfov_deg: float, resolution: Tuple[int, int]) -> float:
    """Horizontal FOV derived from the lens's diagonal FOV spec.

    Camera datasheets (e.g. the Logitech C920/C920s: "campo de visao
    diagonal fixo de 78 graus") commonly quote the DIAGONAL field of view,
    not the horizontal one. For a rectilinear lens sharing one focal length
    across the sensor, the half-angle tangent scales linearly with pixel
    distance from the image center along any axis, so the horizontal
    half-angle tangent is the diagonal half-angle tangent scaled by
    width_px / diagonal_px (same relation `camera_footprint` uses to go
    from horizontal to vertical FOV).

    Args:
        dfov_deg: Diagonal field of view of the lens, in degrees, as quoted
            by the camera datasheet.
        resolution: (width_px, height_px) of the photos this camera produces.

    Returns:
        Horizontal field of view, in degrees.
    """
    width_px, height_px = resolution
    diagonal_px = math.hypot(width_px, height_px)
    half_dfov = math.radians(dfov_deg) / 2

    half_hfov = math.atan(math.tan(half_dfov) * (width_px / diagonal_px))
    return 2 * math.degrees(half_hfov)


def camera_footprint(
    altitude_m: float, hfov_deg: float, resolution: Tuple[int, int]
) -> Tuple[float, float]:
    """Ground footprint (footprint_x_m, footprint_y_m) of a single photo.

    footprint_x follows the image width axis (given hfov_deg). footprint_y
    is derived from the sensor aspect ratio, since a rectilinear lens has
    a single focal length shared by both axes:
    tan(vfov/2) = tan(hfov/2) * (height_px / width_px).

    Args:
        altitude_m: Camera height above the ground plane, in meters.
        hfov_deg: Horizontal field of view of the camera, in degrees.
        resolution: (width_px, height_px) of the photo the footprint is for.

    Returns:
        (footprint_x_m, footprint_y_m): ground footprint size, in meters,
        along the image-width and image-height axes respectively.
    """
    width_px, height_px = resolution
    half_hfov = math.radians(hfov_deg) / 2

    footprint_x = 2 * altitude_m * math.tan(half_hfov)

    half_vfov = math.atan(math.tan(half_hfov) * (height_px / width_px))
    footprint_y = 2 * altitude_m * math.tan(half_vfov)

    return footprint_x, footprint_y


def _axis_positions(span_m: float, footprint_m: float, margin_m: float) -> List[float]:
    """Minimal set of evenly spaced capture-center offsets (from 0, the
    arena/takeoff center) along one axis that gives 100% coverage of
    `span_m`, reaching `margin_m` past each edge, with no gaps between
    adjacent photo footprints.

    Args:
        span_m: Length of the arena along this axis, in meters.
        footprint_m: Length of a single photo's ground footprint along
            this axis, in meters.
        margin_m: Extra distance the coverage must reach past each edge
            of `span_m`, in meters.

    Returns:
        Capture-center offsets along this axis, in meters, relative to 0
        (the arena/takeoff center).
    """
    if footprint_m <= 0:
        raise ValueError('footprint_m must be positive')

    required_reach = span_m / 2 + margin_m

    n = 1
    while True:
        if n == 1:
            spacing = 0.0
            reach = footprint_m / 2
        else:
            spacing = (required_reach - footprint_m / 2) * 2 / (n - 1)
            spacing = min(max(spacing, 0.0), footprint_m)  # cap: never leave a gap mid-span
            reach = (n - 1) / 2 * spacing + footprint_m / 2

        if reach >= required_reach - 1e-9:
            return [(i - (n - 1) / 2) * spacing for i in range(n)]

        n += 1


def compute_grid(
    arena_size_x_m: float,
    arena_size_y_m: float,
    altitude_m: float,
    hfov_deg: float,
    resolution: Tuple[int, int],
    overlap_margin_m: float = 0.3,
    yaw_deg: float = 0.0,
    camera_yaw_offset_deg: float = 0.0,
) -> List[Waypoint]:
    """Compute the minimal grid of capture positions guaranteeing 100%
    coverage of the arena for the given camera/altitude, ordered as a
    serpentine (boustrophedon) path to minimize travel between shots.

    Positions are relative to the arena center (== takeoff position, per
    the competition rules), so they map directly to
    MoveReference.TAKEOFF offsets.

    `camera_yaw_offset_deg` is the same physical mount rotation used by
    `utils.geo_projection.pixel_to_local` (see `camera.mount.yaw_offset_deg`
    in config.yml). At offset 0 the image's wide (HFOV) axis is assumed to
    cover the arena's X span and the narrow (VFOV) axis the Y span; at a
    quarter turn (+-90/270 deg) those are swapped, since the camera's wide
    axis then actually sweeps the arena's Y span instead. Only quarter-turn
    offsets keep the footprint axis-aligned with the arena, which is what
    this rectangular grid requires.

    Args:
        arena_size_x_m: Arena extent along the local X axis, in meters.
        arena_size_y_m: Arena extent along the local Y axis, in meters.
        altitude_m: Flight altitude for the coverage pass, in meters.
        hfov_deg: Horizontal field of view of the camera, in degrees.
        resolution: (width_px, height_px) of the photos the camera produces.
        overlap_margin_m: Extra distance coverage must reach past each
            arena edge, in meters.
        yaw_deg: Yaw, in degrees, assigned to every generated waypoint.
        camera_yaw_offset_deg: Camera mount rotation relative to the drone
            body, in degrees (`camera.mount.yaw_offset_deg` in config.yml);
            only quarter-turn multiples (0/90/180/270) are supported.

    Returns:
        Waypoints covering the arena, ordered along a serpentine
        (boustrophedon) path, with positions relative to the arena center.
    """
    footprint_x, footprint_y = camera_footprint(altitude_m, hfov_deg, resolution)

    quarter_turns = camera_yaw_offset_deg / 90.0
    nearest_quarter_turn = round(quarter_turns)
    if not math.isclose(quarter_turns, nearest_quarter_turn, abs_tol=1e-6):
        raise ValueError(
            'compute_grid only supports camera_yaw_offset_deg at quarter-turn '
            'multiples (0/90/180/270) -- an arbitrarily rotated camera footprint '
            'is not axis-aligned and this rectangular grid cannot cover the '
            'arena for it.'
        )
    if nearest_quarter_turn % 2 != 0:
        footprint_x, footprint_y = footprint_y, footprint_x

    xs = _axis_positions(arena_size_x_m, footprint_x, overlap_margin_m)
    ys = _axis_positions(arena_size_y_m, footprint_y, overlap_margin_m)

    waypoints = []
    for row_idx, y in enumerate(ys):
        row_xs = xs if row_idx % 2 == 0 else list(reversed(xs))
        for x in row_xs:
            waypoints.append(Waypoint(x=x, y=y, z=altitude_m, yaw_deg=yaw_deg))

    return waypoints
