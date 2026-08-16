import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mapping.config import Config
from mapping.utils.coverage import compute_grid


class PlanCoverage(State):
    """Computes the flight-grid waypoints that give 100% camera coverage
    of the arena, from the camera/arena/mission parameters in config.yml.
    """

    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('PLANNING COVERAGE GRID...')

        try:
            waypoints = compute_grid(
                arena_size_x_m=self.config.arena.size_x_m,
                arena_size_y_m=self.config.arena.size_y_m,
                altitude_m=self.config.takeoff_altitude,
                hfov_deg=self.config.camera.hfov_deg,
                resolution=self.config.camera.resolution,
                overlap_margin_m=self.config.mission.overlap_margin_m,
                camera_yaw_offset_deg=self.config.camera.mount.yaw_offset_deg,
            )

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'COVERAGE PLANNING FAILED: {error}')
            return ABORT

        blackboard.set('waypoints', waypoints)
        blackboard.set('waypoint_idx', 0)
        blackboard.set('captures', [])

        yasmin.YASMIN_LOG_INFO(f'\033[32mComputed {len(waypoints)} capture waypoints\033[0m')
        return SUCCEED
