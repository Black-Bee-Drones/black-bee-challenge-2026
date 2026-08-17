import os
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - only missing outside a sourced ROS2 env
    get_package_share_directory = None

from mapping.utils.coverage import hfov_from_dfov


class LandingMode(str, Enum):
    LAND = 'LAND'
    RTL = 'RTL'


class PoseSourceOption(str, Enum):
    GPS = 'gps'
    VISION = 'vision'


@dataclass(frozen=True)
class CameraMountConfig:
    forward_m: float = 0.0
    right_m: float = 0.0
    up_m: float = 0.0
    yaw_offset_deg: float = 0.0


@dataclass(frozen=True)
class DetectionCameraOverride:
    """Resolution/HFOV of the camera actually feeding DetectBases, when it
    differs from the deployment camera (`CameraConfig` above) that
    `compute_grid()` plans the flight around -- e.g. in simulation, where a
    much wider-angle, lower-res sensor stands in for the real field camera.
    Flight planning must stay on the real camera's numbers (otherwise a
    wide simulated lens convinces compute_grid() that 1-2 waypoints already
    "cover" the arena, and the drone barely moves); detection must use
    whatever camera is actually producing the pixels it's measuring.
    """

    resolution: Tuple[int, int]
    hfov_deg: float


@dataclass(frozen=True)
class CameraConfig:
    source: str
    resolution: Tuple[int, int]
    hfov_deg: float  # derived from config.yml's dfov_deg, see Config.load()
    mount: CameraMountConfig
    detection_override: Optional[DetectionCameraOverride] = None
    # Only consumed if source == "c920" (nectar's C920Cam driver), for the
    # rare case its v4l2-ctl auto-detection can't find the device -- see
    # capture_waypoint.py::_ensure_camera().
    c920_fallback_device_index: int = 0


@dataclass(frozen=True)
class ColorCorrectionConfig:
    enabled: bool
    gray_world_white_balance: bool
    gamma: float


@dataclass(frozen=True)
class CalibrationConfig:
    camera_matrix_path: Optional[str]
    distortion_path: Optional[str]
    color_correction: ColorCorrectionConfig


@dataclass(frozen=True)
class GpsPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class ArenaVertices:
    """The 4 arena corners, named exactly like competition organizers call
    them out on the day (A/B/C/D), relative to the drone's nose at takeoff --
    not compass directions, which aren't known until takeoff itself.
    """

    a: GpsPoint  # front-left
    b: GpsPoint  # front-right
    c: GpsPoint  # back-left
    d: GpsPoint  # back-right


@dataclass(frozen=True)
class ArenaConfig:
    size_x_m: float
    size_y_m: float
    vertices_gps: ArenaVertices


@dataclass(frozen=True)
class MissionConfig:
    overlap_margin_m: float
    photos_per_waypoint: int
    stabilize_seconds: float
    move_precision_m: float
    move_timeout_s: float


@dataclass(frozen=True)
class DetectionConfig:
    method: str  # "opencv" (threshold+contour) | "ia" (modelo YOLO)
    model_path: Optional[str]
    ai_confidence: float
    base_size_m: float
    area_tolerance: float
    white_threshold: int
    dedup_radius_m: float
    templates_dir: Optional[str]
    max_bases: int
    # Lê roll/pitch reais do MAVROS (/mavros/local_position/pose) e corrige
    # pixel_to_local() por interseção raio-solo em vez de assumir câmera
    # nadir -- default False porque o sinal de roll/pitch vindo do
    # quaternion ENU do MAVROS ainda não foi validado empiricamente contra
    # um ground truth (mesmo tipo de verificação já feita pro
    # yaw_offset_deg -- ver docs/decisions/0001, adendo 2026-08-17).
    tilt_compensation: bool
    # Se achar menos que max_bases, tenta de novo nas MESMAS fotos já
    # corrigidas (sem recapturar nem costurar um mosaico -- ver adendo
    # 2026-08-17 sobre por que não usar o mosaico pra detecção) com
    # limiares relaxados, só pra preencher a diferença. Ver
    # states/mission/detect_bases.py.
    retry_on_shortfall: bool


@dataclass(frozen=True)
class OutputConfig:
    directory: Optional[str]
    publish_topic: str
    save_report: bool


@dataclass(frozen=True)
class MosaicConfig:
    enabled: bool


def _apply_profile(data: dict) -> dict:
    """Overlay config.yml's profiles[active_profile] on top of the shared
    sections it targets (drone/simulation/camera/arena), so the rest of
    Config.load() reads a fully-resolved dict regardless of which profile
    is active -- see config.yml's `active_profile`/`profiles` for what
    each profile overrides (sim vs. real-flight camera/connection/arena
    values).

    Args:
        data: Raw dict parsed from config.yml (before dataclass
            construction), containing the optional 'active_profile' key,
            the 'profiles' mapping, and the shared config sections
            ('drone', 'simulation', 'camera', 'arena', ...) to overlay.

    Returns:
        The same dict with the active profile's overrides merged into its
        target sections (the dict is also mutated in place); returned
        unchanged if 'active_profile' is falsy.
    """
    profile_name = data.get('active_profile')
    if not profile_name:
        return data
    profiles = data.get('profiles', {})
    if profile_name not in profiles:
        raise KeyError(
            f'active_profile {profile_name!r} not found under config.yml profiles '
            f'(available: {sorted(profiles)})'
        )
    for section, overrides in profiles[profile_name].items():
        data[section] = {**data.get(section, {}), **overrides}
    return data


@dataclass(frozen=True)
class Config:
    drone_type: str
    connection_string: str
    pose_source: PoseSourceOption
    start_driver: bool
    sim_mode: bool
    takeoff_altitude: float
    landing_mode: LandingMode

    camera: CameraConfig
    calibration: CalibrationConfig
    arena: ArenaConfig
    mission: MissionConfig
    detection: DetectionConfig
    output: OutputConfig
    mosaic: MosaicConfig

    @classmethod
    def load(cls, filepath: Optional[str] = None) -> 'Config':
        """Load, profile-resolve, and validate config.yml into a frozen Config.

        Args:
            filepath: Path to the config.yml file to load. Defaults to
                `_default_config_path()` (installed share dir, falling back
                to the source tree) when omitted.

        Returns:
            A fully populated, frozen Config instance built from the file's
            contents, with the active profile's overrides already applied.
        """
        if filepath is None:
            filepath = cls._default_config_path()

        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        data = _apply_profile(data)

        camera_data = data['camera']
        mount_data = camera_data.get('mount', {})
        camera_resolution = tuple(camera_data['resolution'])
        override_data = camera_data.get('detection_override')
        detection_override = None
        if override_data:
            override_resolution = tuple(override_data['resolution'])
            detection_override = DetectionCameraOverride(
                resolution=override_resolution,
                hfov_deg=hfov_from_dfov(override_data['dfov_deg'], override_resolution),
            )

        camera = CameraConfig(
            source=camera_data['source'],
            resolution=camera_resolution,
            hfov_deg=hfov_from_dfov(camera_data['dfov_deg'], camera_resolution),
            mount=CameraMountConfig(
                forward_m=mount_data.get('forward_m', 0.0),
                right_m=mount_data.get('right_m', 0.0),
                up_m=mount_data.get('up_m', 0.0),
                yaw_offset_deg=mount_data.get('yaw_offset_deg', 0.0),
            ),
            detection_override=detection_override,
            c920_fallback_device_index=camera_data.get('c920_fallback_device_index', 0),
        )

        calibration_data = data['calibration']
        color_data = calibration_data.get('color_correction', {})
        calibration = CalibrationConfig(
            camera_matrix_path=calibration_data.get('camera_matrix_path') or None,
            distortion_path=calibration_data.get('distortion_path') or None,
            color_correction=ColorCorrectionConfig(
                enabled=color_data.get('enabled', True),
                gray_world_white_balance=color_data.get('gray_world_white_balance', True),
                gamma=color_data.get('gamma', 1.0),
            ),
        )

        arena_data = data['arena']
        vertices_data = arena_data['vertices_gps']
        arena = ArenaConfig(
            size_x_m=arena_data['size_x_m'],
            size_y_m=arena_data['size_y_m'],
            vertices_gps=ArenaVertices(
                a=GpsPoint(**vertices_data['A']),
                b=GpsPoint(**vertices_data['B']),
                c=GpsPoint(**vertices_data['C']),
                d=GpsPoint(**vertices_data['D']),
            ),
        )

        mission_data = data['mission']
        mission = MissionConfig(
            overlap_margin_m=mission_data['overlap_margin_m'],
            photos_per_waypoint=mission_data['photos_per_waypoint'],
            stabilize_seconds=mission_data['stabilize_seconds'],
            move_precision_m=mission_data['move_precision_m'],
            move_timeout_s=mission_data['move_timeout_s'],
        )

        detection_data = data['detection']
        detection = DetectionConfig(
            method=detection_data.get('method', 'opencv'),
            model_path=detection_data.get('model_path') or None,
            ai_confidence=detection_data.get('ai_confidence', 0.5),
            base_size_m=detection_data['base_size_m'],
            area_tolerance=detection_data['area_tolerance'],
            white_threshold=detection_data['white_threshold'],
            dedup_radius_m=detection_data['dedup_radius_m'],
            templates_dir=detection_data.get('templates_dir') or None,
            max_bases=detection_data['max_bases'],
            tilt_compensation=detection_data.get('tilt_compensation', False),
            retry_on_shortfall=detection_data.get('retry_on_shortfall', True),
        )

        output_data = data['output']
        output = OutputConfig(
            directory=output_data.get('directory') or None,
            publish_topic=output_data['publish_topic'],
            save_report=output_data.get('save_report', True),
        )

        mosaic = MosaicConfig(enabled=data.get('mosaic', {}).get('enabled', False))

        return cls(
            drone_type=data['drone']['type'],
            connection_string=data['drone']['connection_string'],
            pose_source=PoseSourceOption(data['drone'].get('pose_source', 'gps')),
            start_driver=data['drone'].get('start_driver', False),
            sim_mode=data['simulation']['mode'],
            takeoff_altitude=data['takeoff']['altitude'],
            landing_mode=LandingMode(data['land']['mode']),
            camera=camera,
            calibration=calibration,
            arena=arena,
            mission=mission,
            detection=detection,
            output=output,
            mosaic=mosaic,
        )

    @staticmethod
    def _default_config_path() -> str:
        """Locate config.yml in the installed package share directory.

        Falls back to the source tree path so `Config.load()` also works
        when running scripts directly out of the workspace (not installed).

        Returns:
            Absolute path to config.yml -- the installed share-dir copy if
            found, otherwise the path inside the source tree.
        """
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory('mapping')
                share_path = os.path.join(share_dir, 'config.yml')
                if os.path.exists(share_path):
                    return share_path
            except Exception:
                pass

        return os.path.join(os.path.dirname(__file__), 'config.yml')


def default_templates_dir() -> str:
    """Locate Simulation/Base_Images, installed or in the source tree.

    Returns:
        Absolute path to the Base_Images directory (shape templates for
        match_shape()) -- the installed share-dir copy if found, otherwise
        the path inside the source tree.
    """
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory('mapping')
            share_path = os.path.join(share_dir, 'Simulation', 'Base_Images')
            if os.path.isdir(share_path):
                return share_path
        except Exception:
            pass

    return os.path.join(os.path.dirname(__file__), '..', 'Simulation', 'Base_Images')


def default_model_path() -> str:
    """Locate the bundled YOLO weights (models/base_detector.pt), installed
    or in the source tree -- same install/source fallback pattern as
    default_templates_dir().

    Returns:
        Absolute path to base_detector.pt (the YOLO weights file) -- the
        installed share-dir copy if found, otherwise the path inside the
        source tree.
    """
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory('mapping')
            share_path = os.path.join(share_dir, 'models', 'base_detector.pt')
            if os.path.exists(share_path):
                return share_path
        except Exception:
            pass

    return os.path.join(os.path.dirname(__file__), 'models', 'base_detector.pt')


# Para usar no código:
# config = Config.load()


def _demo():
    # ponytail self-check: _apply_profile() overlay logic, no file I/O.
    base = {
        'active_profile': 'simulation',
        'drone': {'connection_string': 'base-value', 'type': 'mavros'},
        'profiles': {
            'simulation': {'drone': {'connection_string': 'sim-value'}},
            'real': {'drone': {'connection_string': 'real-value'}},
        },
    }

    sim = _apply_profile({**base, 'active_profile': 'simulation'})
    assert sim['drone']['connection_string'] == 'sim-value', sim
    assert sim['drone']['type'] == 'mavros', sim  # untouched key preserved

    real = _apply_profile({**base, 'active_profile': 'real'})
    assert real['drone']['connection_string'] == 'real-value', real

    try:
        _apply_profile({**base, 'active_profile': 'nonexistent'})
        assert False, 'expected KeyError for unknown active_profile'
    except KeyError:
        pass

    print('config profile self-check OK')


if __name__ == '__main__':
    _demo()
