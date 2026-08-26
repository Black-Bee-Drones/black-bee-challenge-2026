import json
import os
from datetime import datetime

import cv2
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from nectar_interfaces.msg import PhotoInfo

from mapping.config import Config
from mapping.utils.geo_projection import LocalToGpsTransform


class PublishResults(State):
    """Publishes one nectar_interfaces/PhotoInfo message per detected base
    (coordinates + photo id) on a ROS topic, and saves the proof photos
    plus a results report to disk -- the two things required for scoring.
    """

    def __init__(self, config: Config):
        """Args:
            config: Loaded mission configuration; used for arena GPS
                transform, output directory/topic/save_report, and
                takeoff_altitude (reported as each base's altitude).
        """
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config
        self.node = YasminNode.get_instance()
        self.publisher = self.node.create_publisher(
            PhotoInfo, config.output.publish_topic, 10
        )

    def execute(self, blackboard: Blackboard):
        """Convert each detected base's local coordinates to GPS, publish a
        PhotoInfo message per base, and save proof photos plus a JSON
        report to disk.

        Args:
            blackboard: Shared mission state. Reads 'base_results' (list of
                BaseResult from DetectBases). Writes 'results_dir' (the
                timestamped output directory used for this run) on success.

        Returns:
            SUCCEED once all results are published and saved; ABORT if an
            exception/KeyboardInterrupt occurs during GPS conversion,
            file I/O, or publishing.
        """
        yasmin.YASMIN_LOG_INFO('PUBLISHING RESULTS...')

        try:
            results = blackboard.get('base_results')

            transform = LocalToGpsTransform(
                self.config.arena.size_x_m,
                self.config.arena.size_y_m,
                self.config.arena.vertices_gps,
            )
            
            if self.config.sim_mode == True:
                output_dir = self.config.output.directory or os.path.expanduser(
                    '~/.ros/mapping_results'
                )
            else:
                output_dir = self.config.output.directory or os.path.expanduser(
                    '~/ros2_ws/src/mapping/results'
                )
            run_dir = os.path.join(output_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
            os.makedirs(run_dir, exist_ok=True)

            report = []
            for i, base in enumerate(results, start=1):
                lat, lon = transform.to_gps(base.local_x, base.local_y)

                photo_num = f'base_{i:02d}'
                photo_path = os.path.join(run_dir, f'{photo_num}.jpg')
                cv2.imwrite(photo_path, base.crop)

                msg = PhotoInfo()
                msg.coordinates = [lat, lon, self.config.takeoff_altitude]
                msg.photo_num = photo_num
                self.publisher.publish(msg)

                report.append(
                    {
                        'lat': lat,
                        'lon': lon,
                        'photo_path': photo_path,
                    }
                )

            if self.config.output.save_report:
                with open(os.path.join(run_dir, 'results.json'), 'w') as f:
                    json.dump(report, f, indent=2)

            blackboard.set('results_dir', run_dir)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'PUBLISH RESULTS FAILED: {error}')
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            f'\033[32mPublished {len(report)} base(s) to '
            f'"{self.config.output.publish_topic}", saved to {run_dir}\033[0m'
        )
        return SUCCEED
