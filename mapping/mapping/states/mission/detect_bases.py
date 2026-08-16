import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mapping.config import Config, default_model_path, default_templates_dir
from mapping.utils.ai_detector import find_base_squares_ai, load_model
from mapping.utils.base_detector import (
    BaseCandidate,
    deduplicate,
    find_base_squares,
    load_shape_templates,
    match_shape,
)
from mapping.utils.geo_projection import compute_gsd, pixel_to_local
from mapping.utils.image_pipeline import correct_color, load_calibration, sharpness_score, undistort


class DetectBases(State):
    """Runs the vision pipeline (undistort -> color correction -> base
    detection -> pixel->local projection) over every captured photo, then
    deduplicates detections of the same physical base across overlapping
    photos.
    """

    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('DETECTING BASES...')

        try:
            captures = blackboard.get('captures')

            camera_matrix, dist_coeffs = load_calibration(self.config.calibration)

            use_ai = self.config.detection.method == 'ia'
            if use_ai:
                model_path = self.config.detection.model_path or default_model_path()
                model = load_model(model_path)
                templates = {}
            else:
                templates_dir = self.config.detection.templates_dir or default_templates_dir()
                templates = load_shape_templates(templates_dir)

            color_cfg = self.config.calibration.color_correction
            candidates = []

            # The camera actually producing these pixels (detection_override
            # in simulation, where it differs from the real deployment
            # camera plan_coverage.py planned the flight around -- see
            # config.yml's comment on camera.detection_override).
            override = self.config.camera.detection_override
            detect_resolution = override.resolution if override else self.config.camera.resolution
            detect_hfov_deg = override.hfov_deg if override else self.config.camera.hfov_deg

            for pose, raw_img in captures:
                # gsd depends on altitude, which is read fresh per photo
                # (capture_waypoint.py's real rangefinder/AGL reading, not a
                # single nominal altitude) -- must be recomputed per photo.
                gsd = compute_gsd(pose.altitude_m, detect_hfov_deg, detect_resolution)
                expected_side_px = self.config.detection.base_size_m / gsd

                img = undistort(raw_img, camera_matrix, dist_coeffs)
                if color_cfg.enabled:
                    img = correct_color(
                        img,
                        gray_world_white_balance=color_cfg.gray_world_white_balance,
                        gamma=color_cfg.gamma,
                    )

                if use_ai:
                    detections = find_base_squares_ai(
                        img, model, confidence=self.config.detection.ai_confidence
                    )
                else:
                    detections = find_base_squares(
                        img,
                        expected_side_px=expected_side_px,
                        area_tolerance=self.config.detection.area_tolerance,
                        white_threshold=self.config.detection.white_threshold,
                    )

                for det in detections:
                    local_x, local_y = pixel_to_local(
                        det.pixel_center[0],
                        det.pixel_center[1],
                        detect_resolution,
                        gsd,
                        pose,
                        camera_yaw_offset_deg=self.config.camera.mount.yaw_offset_deg,
                    )
                    shape_label = det.shape_label
                    if shape_label is None and not use_ai:
                        shape_label = match_shape(det.shape_contour, det.digit_contour, templates)
                    candidates.append(
                        BaseCandidate(
                            local_x=local_x,
                            local_y=local_y,
                            detection=det,
                            sharpness=sharpness_score(det.crop),
                            shape_label=shape_label,
                        )
                    )

            results = deduplicate(
                candidates,
                radius_m=self.config.detection.dedup_radius_m,
                max_bases=self.config.detection.max_bases,
            )

            blackboard.set('base_results', results)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'BASE DETECTION FAILED: {error}')
            return ABORT

        yasmin.YASMIN_LOG_INFO(f'\033[32mFound {len(results)} candidate base(s)\033[0m')
        return SUCCEED
