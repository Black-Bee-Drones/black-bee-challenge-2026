import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from nectar.control import AltitudeSource, MavrosDrone, MoveReference
from nectar.vision import ImageHandler

from mapping.config import Config
from mapping.utils.geo_projection import CapturePose
from mapping.utils.image_pipeline import pick_sharpest

NEXT = 'next'


class CaptureWaypoint(State):
    """Flies to each planned waypoint (relative to the takeoff/arena
    center) and captures the sharpest of N photos there. Self-loops
    (outcome NEXT) until every waypoint has been visited.
    """

    def __init__(self, config: Config):
        super().__init__(outcomes=[NEXT, SUCCEED, ABORT])
        self.config = config
        self.node = YasminNode.get_instance()
        self.image_handler = None
        self.takeoff_heading = None

    def _ensure_camera(self):
        if self.image_handler is None:
            self.image_handler = ImageHandler(
                image_source=self.config.camera.source,
            )
            self.image_handler.open()

    @staticmethod
    def _safe_heading(drone: MavrosDrone, default: float = 0.0) -> float:
        """`heading` is only populated outdoors (PoseSource.GPS); fall back
        to `default` (keeping the local frame yaw-locked) otherwise.
        """
        try:
            return drone.heading
        except Exception:
            return default

    def execute(self, blackboard: Blackboard):
        self._ensure_camera()

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

            pose = CapturePose(
                local_x=waypoint.x,
                local_y=waypoint.y,
                altitude_m=current_altitude,
                heading_offset_deg=heading_offset,
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
