import math
from typing import Optional, Tuple

import yasmin
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_ros.yasmin_node import YasminNode

from nectar.control import AltitudeSource, MavrosDrone, MoveReference
from nectar.vision import C920Config, ImageHandler

from mapping.config import Config
from mapping.utils.geo_projection import CapturePose
from mapping.utils.image_pipeline import pick_sharpest

NEXT = 'next'

# nectar-sdk's C920Cam only supports these 3 fixed capture profiles (see
# C920Config.profile) -- camera.resolution in config.yml must match one
# exactly when source == "c920", or the real camera silently captures at
# a different resolution than every GSD/pixel calculation downstream
# assumes (detect_bases.py, plan_coverage.py) is built around.
_C920_PROFILE_RESOLUTIONS = {0: (640, 480), 1: (1280, 720), 2: (1920, 1080)}


def _c920_profile_for(resolution: Tuple[int, int]) -> int:
    """Map a configured camera.resolution to the matching C920Cam capture
    profile index.

    Args:
        resolution: (width_px, height_px) requested in config.yml's
            camera.resolution.

    Returns:
        The C920Config.profile index (0, 1, or 2) whose fixed resolution
        matches `resolution` exactly.
    """
    for profile, profile_resolution in _C920_PROFILE_RESOLUTIONS.items():
        if profile_resolution == tuple(resolution):
            return profile
    raise ValueError(
        f'camera.resolution {tuple(resolution)!r} has no matching C920Cam profile '
        f'(supported: {list(_C920_PROFILE_RESOLUTIONS.values())}) -- pick one of '
        f'those for camera.resolution, or use source: "webcam" for a non-C920 USB camera.'
    )


class CaptureWaypoint(State):
    """Flies to each planned waypoint (relative to the takeoff/arena
    center) and captures the sharpest of N photos there. Self-loops
    (outcome NEXT) until every waypoint has been visited.
    """

    def __init__(self, config: Config):
        """Args:
            config: Loaded mission configuration; used for camera setup,
                mission timing/precision, and detection.tilt_compensation.
        """
        super().__init__(outcomes=[NEXT, SUCCEED, ABORT])
        self.config = config
        self.node = YasminNode.get_instance()
        self.image_handler = None
        self.takeoff_heading = None
        self._attitude_sub = None
        # Raw (roll, pitch) in radians, MAVROS's own ENU/FLU convention
        # (REP-103) -- converted to the FRD sign pixel_to_local() expects
        # in _tilt_deg() below, not here.
        self._latest_attitude_rad: Optional[Tuple[float, float]] = None

    def _ensure_camera(self) -> None:
        if self.image_handler is None:
            camera_config = None
            if self.config.camera.source == 'c920':
                # Auto-detected by ImageHandler otherwise (config=None) --
                # but nectar's default C920Config().profile is 1 (1280x720),
                # not necessarily camera.resolution here, so it must be
                # passed explicitly for the real camera to actually capture
                # at the resolution the rest of the pipeline assumes.
                camera_config = C920Config(
                    profile=_c920_profile_for(self.config.camera.resolution),
                    fallback_device_index=self.config.camera.c920_fallback_device_index,
                )
            self.image_handler = ImageHandler(
                image_source=self.config.camera.source,
                config=camera_config,
            )
            self.image_handler.open()

    def _ensure_attitude_sub(self) -> None:
        # nectar-sdk doesn't expose roll/pitch for the MAVROS backend
        # (MavrosTransport never populates VehicleTransport.attitude, only
        # local position/yaw) -- read the raw topic directly instead of
        # waiting on an SDK change. Only subscribed when
        # detection.tilt_compensation is on, since it's otherwise unused.
        if self._attitude_sub is None:
            self._attitude_sub = self.node.create_subscription(
                PoseStamped,
                '/mavros/local_position/pose',
                self._on_local_pose,
                qos_profile_sensor_data,
            )

    def _on_local_pose(self, msg: PoseStamped) -> None:
        """Subscription callback: cache the latest roll/pitch reading.

        Args:
            msg: Latest `/mavros/local_position/pose` message; only its
                orientation quaternion is used.
        """
        q = msg.pose.orientation
        roll, pitch, _yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._latest_attitude_rad = (roll, pitch)

    def _tilt_deg(self) -> Tuple[float, float]:
        """(roll_deg, pitch_deg) in the aviation/FRD sign pixel_to_local()
        expects, or (0.0, 0.0) if disabled/not yet received a reading.

        MAVROS publishes `/mavros/local_position/pose` in ENU/FLU
        (REP-103): X=forward, Y=left, Z=up. pixel_to_local()'s ray-ground
        model uses FRD (X=forward, Y=right, Z=down), matching
        mount_forward_m/right_m/up_m. Flipping Y and Z leaves a rotation
        about the shared X axis (roll) with the same sign, but flips the
        sign of a rotation about Y (pitch) -- same "roll unchanged, pitch
        negated" conversion nectar-sdk's own mavlink transport already
        applies for an analogous FRD conversion (see
        nectar/control/mavlink/transport.py's `_on_attitude`).
        NOT YET EMPIRICALLY VERIFIED against a ground truth for this
        specific MAVROS/ArduPilot setup -- see config.yml's
        detection.tilt_compensation comment before enabling for a real
        mission.

        Returns:
            (roll_deg, pitch_deg) tuple in degrees, FRD/aviation sign
            convention for pixel_to_local(); (0.0, 0.0) if
            detection.tilt_compensation is off or no attitude reading has
            arrived yet.
        """
        if not self.config.detection.tilt_compensation or self._latest_attitude_rad is None:
            return 0.0, 0.0
        roll_flu, pitch_flu = self._latest_attitude_rad
        return math.degrees(roll_flu), math.degrees(-pitch_flu)

    @staticmethod
    def _safe_heading(drone: MavrosDrone, default: float = 0.0) -> float:
        """`heading` is only populated outdoors (PoseSource.GPS); fall back
        to `default` (keeping the local frame yaw-locked) otherwise.

        Args:
            drone: Connected drone whose `.heading` property is queried.
            default: Heading in degrees to fall back to when `.heading`
                raises (e.g. indoors / vision pose source).

        Returns:
            The drone's current heading in degrees, or `default` if
            unavailable.
        """
        try:
            return drone.heading
        except Exception:
            return default

    def execute(self, blackboard: Blackboard) -> str:
        """Fly to the current waypoint, capture the sharpest of N photos,
        and record its CapturePose. Self-loops until every waypoint in
        'waypoints' has been visited.

        Args:
            blackboard: Shared mission state. Reads 'waypoints' (grid from
                PlanCoverage), 'waypoint_idx' (current index), 'drone', and
                'captures' (list accumulated so far). Writes 'waypoint_idx'
                (incremented) and 'captures' (appends the new
                (CapturePose, photo) tuple) on success.

        Returns:
            NEXT if waypoints remain after this one; SUCCEED once the last
            waypoint has been captured (camera is also cleaned up in this
            case); ABORT if no photo could be captured at this waypoint, or
            if an exception/KeyboardInterrupt occurs while moving/capturing.
        """
        self._ensure_camera()
        if self.config.detection.tilt_compensation:
            self._ensure_attitude_sub()

        waypoints = blackboard.get('waypoints')
        idx = blackboard.get('waypoint_idx')
        waypoint = waypoints[idx]

        yasmin.YASMIN_LOG_INFO(
            f'CAPTURE WAYPOINT {idx + 1}/{len(waypoints)}: '
            f'x={waypoint.x:.2f}m y={waypoint.y:.2f}m (from takeoff)'
        )

        drone: MavrosDrone = blackboard.get('drone')

        try:
            if self.takeoff_heading is None:
                self.takeoff_heading = self._safe_heading(drone)

            reached = drone.move_to(
                x=waypoint.x,
                y=waypoint.y,
                z=0.0,
                reference=MoveReference.TAKEOFF,
                precision=self.config.mission.move_precision_m,
                timeout=self.config.mission.move_timeout_s,
            )
            if not reached:
                yasmin.YASMIN_LOG_WARN(
                    f'Waypoint {idx} not reached within precision, capturing anyway'
                )

            drone.delay(self.config.mission.stabilize_seconds)

            photos = []
            for _ in range(self.config.mission.photos_per_waypoint):
                photo = self.image_handler.take_photo(timeout_sec=2.0)
                if photo is not None:
                    photos.append(photo)

            best_photo, score = pick_sharpest(photos)
            if best_photo is None:
                yasmin.YASMIN_LOG_ERROR(f'No photo captured at waypoint {idx}')
                return ABORT

            current_heading = self._safe_heading(drone, default=self.takeoff_heading)
            heading_offset = (current_heading - self.takeoff_heading + 180) % 360 - 180

            # Real AGL reading (rangefinder) at capture time, not the nominal
            # takeoff altitude -- PID overshoot/settling means actual altitude
            # drifts a bit per waypoint, which skews ground-sample-distance
            # (and therefore every detected base's position) if left static.
            current_altitude = drone.get_altitude(AltitudeSource.LIDAR)
            if current_altitude is None:
                current_altitude = drone.get_altitude(AltitudeSource.REL_ALT)
            if current_altitude is None:
                current_altitude = self.config.takeoff_altitude

            roll_deg, pitch_deg = self._tilt_deg()
            pose = CapturePose(
                local_x=waypoint.x,
                local_y=waypoint.y,
                altitude_m=current_altitude,
                heading_offset_deg=heading_offset,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
            )

            captures = blackboard.get('captures')
            captures.append((pose, best_photo))
            blackboard.set('captures', captures)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'CAPTURE WAYPOINT FAILED: {error}')
            return ABORT

        idx += 1
        blackboard.set('waypoint_idx', idx)

        if idx < len(waypoints):
            return NEXT

        self.image_handler.cleanup()
        yasmin.YASMIN_LOG_INFO('\033[32mAll waypoints captured!\033[0m')
        return SUCCEED


def _demo() -> None:
    # ponytail self-check: _c920_profile_for()'s resolution -> profile
    # mapping, no ROS/camera hardware needed.
    assert _c920_profile_for((1920, 1080)) == 2
    assert _c920_profile_for([1280, 720]) == 1
    assert _c920_profile_for((640, 480)) == 0
    try:
        _c920_profile_for((800, 600))
        assert False, 'expected ValueError for an unsupported resolution'
    except ValueError:
        pass
    print('capture_waypoint self-check OK')


if __name__ == '__main__':
    _demo()
