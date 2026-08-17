import math
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

import numpy as np
import yasmin
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from mapping.config import Config, default_model_path, default_templates_dir
from mapping.utils.ai_detector import find_base_squares_ai, load_model
from mapping.utils.base_detector import (
    BaseCandidate,
    BaseResult,
    centrality_weight,
    deduplicate,
    Detection,
    find_base_squares,
    load_shape_templates,
    match_shape,
)
from mapping.utils.geo_projection import CapturePose, compute_gsd, pixel_to_local
from mapping.utils.image_pipeline import correct_color, load_calibration, sharpness_score, undistort
from mapping.utils.mosaic import canvas_to_local, merge_base_crop

# CorrectedPhoto: (pose, undistorted+color-corrected img, gsd_m_per_px,
# camera_altitude_m) -- cached per photo in DetectBases.execute() so the
# shortfall retry pass (detection.retry_on_shortfall) can re-run detection
# without redoing undistort()/correct_color() on all 28-ish photos again.
CorrectedPhoto = Tuple[CapturePose, np.ndarray, float, float]

# "Try harder" deltas for the shortfall retry pass, applied to whichever
# thresholds config.yml's detection.* already set -- not exposed as their
# own config.yml fields on purpose: they're an implementation detail of
# "look again at photos we already have", not something a field operator
# would tune per mission (detection.retry_on_shortfall is the only knob).
_RETRY_WHITE_THRESHOLD_DELTA = 40
_RETRY_WHITE_THRESHOLD_MIN = 100
_RETRY_AREA_TOLERANCE_DELTA = 0.25
_RETRY_AI_CONFIDENCE_DELTA = 0.2
_RETRY_AI_CONFIDENCE_MIN = 0.15

# Window merge_base_crop()'s composite canvas covers around a base, as a
# fraction of base_size_m -- passed explicitly (instead of relying on
# merge_base_crop()'s own default) since _recover_edge_cut_bases() needs
# the exact same value to convert a pixel in that canvas back to local
# coordinates via canvas_to_local().
_MERGE_MARGIN_FRAC = 0.5


def _drop_candidates_near(
    candidates: List[BaseCandidate], results: List[BaseResult], radius_m: float
) -> List[BaseCandidate]:
    """Candidates from the shortfall retry pass that land within
    `radius_m` of an already-found base are just weaker re-detections of
    it, not a new base -- drop them so they can't steal one of the
    `shortfall` retry slots from an actually-new base.

    Args:
        candidates: New candidates from a retry/recovery pass, in local
            meters.
        results: Bases already confirmed in a prior pass, to check
            proximity against.
        radius_m: Distance in meters within which a candidate is
            considered the same physical base as an existing result.

    Returns:
        The subset of `candidates` that is farther than `radius_m` from
        every existing result.
    """
    return [
        c
        for c in candidates
        if not any(
            math.hypot(c.local_x - r.local_x, c.local_y - r.local_y) <= radius_m for r in results
        )
    ]


class BaseFinder(ABC):
    """Contract for locating candidate base squares in an already
    undistorted/color-corrected photo -- implemented by the OpenCV
    threshold+contour path (`OpenCVBaseFinder`) and the YOLO path
    (`AIBaseFinder`), chosen once per mission by `DetectBases` based on
    `config.yml`'s `detection.method`.
    """

    @abstractmethod
    def find(self, img: np.ndarray, gsd_m_per_px: float) -> List[Detection]:
        """Detect candidate bases in `img`, given the photo's ground sample distance.

        Args:
            img: Undistorted, color-corrected photo (BGR).
            gsd_m_per_px: Ground sample distance in meters/pixel at this
                photo's altitude, used to convert base_size_m to an
                expected pixel size.

        Returns:
            One Detection per candidate base square found in `img`.
        """

    @abstractmethod
    def find_partial_at_edge(self, img: np.ndarray, gsd_m_per_px: float) -> List[Detection]:
        """Detect bases cut off by the image border too heavily for
        `find()` to have accepted them at all -- for
        DetectBases._recover_edge_cut_bases() to cluster across photos and
        composite with merge_base_crop(), then re-validate with `find()`
        on the result. Each returned Detection has fully_in_frame=False by
        construction.

        Args:
            img: Undistorted, color-corrected photo (BGR).
            gsd_m_per_px: Ground sample distance in meters/pixel at this
                photo's altitude.

        Returns:
            One Detection per heavily edge-cut candidate found in `img`
            (empty if the implementation has no edge-recovery path).
        """


class OpenCVBaseFinder(BaseFinder):
    def __init__(self, base_size_m: float, area_tolerance: float, white_threshold: int) -> None:
        """Args:
            base_size_m: Physical side length of a base marker, in meters
                (detection.base_size_m).
            area_tolerance: Fractional tolerance around the expected
                contour area/aspect for a candidate to be accepted
                (detection.area_tolerance).
            white_threshold: Grayscale threshold (0-255) above which a
                pixel counts as the base's white background
                (detection.white_threshold).
        """
        self._base_size_m = base_size_m
        self._area_tolerance = area_tolerance
        self._white_threshold = white_threshold

    def find(self, img: np.ndarray, gsd_m_per_px: float) -> List[Detection]:
        """Threshold+contour detection of fully-in-frame base candidates.

        Args:
            img: Undistorted, color-corrected photo (BGR).
            gsd_m_per_px: Ground sample distance in meters/pixel, used to
                convert base_size_m into the expected contour side length
                in pixels.

        Returns:
            One Detection per candidate base square passing the
            area/aspect/white-threshold filters.
        """
        expected_side_px = self._base_size_m / gsd_m_per_px
        return find_base_squares(
            img,
            expected_side_px=expected_side_px,
            area_tolerance=self._area_tolerance,
            white_threshold=self._white_threshold,
        )

    def find_partial_at_edge(self, img: np.ndarray, gsd_m_per_px: float) -> List[Detection]:
        """Threshold+contour detection relaxed to also accept contours
        touching the image border, for bases cut off too heavily for
        `find()` to accept at all.

        Args:
            img: Undistorted, color-corrected photo (BGR).
            gsd_m_per_px: Ground sample distance in meters/pixel.

        Returns:
            Only the newly-accepted, edge-touching Detections (the ones
            `find()` would have rejected); each has fully_in_frame=False.
        """
        expected_side_px = self._base_size_m / gsd_m_per_px
        detections = find_base_squares(
            img,
            expected_side_px=expected_side_px,
            area_tolerance=self._area_tolerance,
            white_threshold=self._white_threshold,
            accept_partial_at_edge=True,
        )
        # find() above (accept_partial_at_edge=False) already covers the
        # fully_in_frame=True and slightly-cut cases; only the ones that
        # find() alone would have rejected are new here.
        return [det for det in detections if not det.fully_in_frame]


class AIBaseFinder(BaseFinder):
    def __init__(self, model_path: str, confidence: float) -> None:
        """Args:
            model_path: Filesystem path to the YOLO weights file
                (detection.model_path, or default_model_path()).
            confidence: Minimum YOLO confidence score (0-1) for a detection
                to be kept (detection.ai_confidence).
        """
        self._model = load_model(model_path)
        self._confidence = confidence

    def find(self, img: np.ndarray, gsd_m_per_px: float) -> List[Detection]:
        """YOLO detection of base candidates.

        Args:
            img: Undistorted, color-corrected photo (BGR).
            gsd_m_per_px: Ground sample distance in meters/pixel (unused by
                the YOLO path itself, kept to satisfy the BaseFinder
                contract).

        Returns:
            One Detection per YOLO box scoring at or above `self._confidence`.
        """
        return find_base_squares_ai(img, self._model, confidence=self._confidence)

    def find_partial_at_edge(self, img: np.ndarray, gsd_m_per_px: float) -> List[Detection]:
        """No-op: YOLO isn't gated by a hard area/aspect window like
        find_base_squares() is, so it already tends to fire (at lower
        confidence) on partial views -- detection.retry_on_shortfall's
        lowered ai_confidence covers that; no separate edge-specific path
        needed here.

        Args:
            img: Unused.
            gsd_m_per_px: Unused.

        Returns:
            Always an empty list.
        """
        # YOLO isn't gated by a hard area/aspect window like find_base_squares()
        # is, so it already tends to fire (at lower confidence) on partial
        # views -- detection.retry_on_shortfall's lowered ai_confidence
        # covers that; no separate edge-specific path needed here.
        return []


class DetectBases(State):
    """Runs the vision pipeline (undistort -> color correction -> base
    detection -> pixel->local projection) over every captured photo, then
    deduplicates detections of the same physical base across overlapping
    photos.
    """

    def __init__(self, config: Config) -> None:
        """Args:
            config: Loaded mission configuration; used throughout for
                detection/calibration/camera parameters.
        """
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def _build_finder(self, relaxed: bool = False) -> BaseFinder:
        """Build the BaseFinder for config.detection.method, optionally
        with relaxed thresholds for the shortfall retry pass.

        Args:
            relaxed: If True, loosen the finder's thresholds (lower
                ai_confidence, or looser area_tolerance/white_threshold)
                per the module-level `_RETRY_*` deltas.

        Returns:
            An AIBaseFinder if detection.method == "ia", otherwise an
            OpenCVBaseFinder.
        """
        detection = self.config.detection
        if detection.method == 'ia':
            model_path = detection.model_path or default_model_path()
            confidence = detection.ai_confidence
            if relaxed:
                confidence = max(_RETRY_AI_CONFIDENCE_MIN, confidence - _RETRY_AI_CONFIDENCE_DELTA)
            return AIBaseFinder(model_path, confidence)

        area_tolerance = detection.area_tolerance
        white_threshold = detection.white_threshold
        if relaxed:
            area_tolerance += _RETRY_AREA_TOLERANCE_DELTA
            white_threshold = max(
                _RETRY_WHITE_THRESHOLD_MIN, white_threshold - _RETRY_WHITE_THRESHOLD_DELTA
            )
        return OpenCVBaseFinder(detection.base_size_m, area_tolerance, white_threshold)

    def _fill_in_incomplete_crop(self, result: BaseResult, resolution: Tuple[int, int]) -> None:
        """If no photo in result.cluster showed the base fully in frame
        (e.g. it sits on the seam between two waypoints' footprints),
        replace result.crop with a composite of every confirming photo
        instead of a single cropped one -- the competition only scores
        photos where the base appears whole. No-op (keeps the sharpest
        partial crop deduplicate() already picked) if any photo did show
        it whole, or if compositing doesn't produce anything usable.

        Args:
            result: The deduplicated base result to (possibly) patch in
                place; its `.crop` is replaced when a composite is built.
            resolution: (width_px, height_px) of the photos in
                result.cluster, used by merge_base_crop() to size the
                composite canvas.

        Returns:
            None. Mutates `result.crop` in place when a usable composite
            is produced.
        """
        if any(c.detection.fully_in_frame for c in result.cluster):
            return
        merged = merge_base_crop(
            [
                (c.pose, c.source_img, c.gsd_m_per_px, c.camera_altitude_m)
                for c in result.cluster
                if c.source_img is not None
            ],
            base_local_x=result.local_x,
            base_local_y=result.local_y,
            base_size_m=self.config.detection.base_size_m,
            resolution=resolution,
            camera_yaw_offset_deg=self.config.camera.mount.yaw_offset_deg,
            mount_forward_m=self.config.camera.mount.forward_m,
            mount_right_m=self.config.camera.mount.right_m,
            margin_frac=_MERGE_MARGIN_FRAC,
        )
        if merged is not None:
            result.crop = merged

    def _detect_in_photo(
        self,
        finder: BaseFinder,
        photo: CorrectedPhoto,
        detect_resolution: Tuple[int, int],
        templates: Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]],
    ) -> List[BaseCandidate]:
        """Run `finder` over one corrected photo and project each
        detection's pixel center to local meters.

        Args:
            finder: BaseFinder to run (`finder.find()`).
            photo: (pose, undistorted+color-corrected image, gsd_m_per_px,
                camera_altitude_m) for this photo.
            detect_resolution: (width_px, height_px) of `photo`'s image,
                used for pixel_to_local() and centrality_weight().
            templates: Shape-name -> (template image, optional digit
                template) map from load_shape_templates(), used to label a
                detection's shape when the finder didn't already supply one.

        Returns:
            One BaseCandidate per detection in `photo`, with local
            coordinates, sharpness, shape label, and centrality weight
            filled in.
        """
        pose, img, gsd, camera_altitude = photo
        candidates = []
        for det in finder.find(img, gsd):
            local_x, local_y = pixel_to_local(
                det.pixel_center[0],
                det.pixel_center[1],
                detect_resolution,
                gsd,
                camera_altitude,
                pose,
                camera_yaw_offset_deg=self.config.camera.mount.yaw_offset_deg,
                mount_forward_m=self.config.camera.mount.forward_m,
                mount_right_m=self.config.camera.mount.right_m,
            )
            shape_label = det.shape_label
            if shape_label is None:
                shape_label = match_shape(det.shape_contour, det.digit_contour, templates)
            # Only kept for photos that don't already show the base whole --
            # see BaseCandidate's docstring comment and
            # _fill_in_incomplete_crop() above. Most photos are
            # fully_in_frame, so this stays cheap in the common case.
            needs_context = not det.fully_in_frame
            candidates.append(
                BaseCandidate(
                    local_x=local_x,
                    local_y=local_y,
                    detection=det,
                    sharpness=sharpness_score(det.crop),
                    shape_label=shape_label,
                    weight=centrality_weight(det.pixel_center, detect_resolution),
                    pose=pose if needs_context else None,
                    gsd_m_per_px=gsd if needs_context else None,
                    camera_altitude_m=camera_altitude if needs_context else None,
                    source_img=img if needs_context else None,
                )
            )
        return candidates

    def _retry_shortfall(
        self,
        corrected_photos: List[CorrectedPhoto],
        results: List[BaseResult],
        detect_resolution: Tuple[int, int],
        templates: Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]],
    ) -> List[BaseResult]:
        """If fewer than detection.max_bases were found, look again at the
        SAME already-captured/corrected photos (no recapture, no mosaic --
        see docs/decisions/0001's 2026-08-17 adendo on why detection never
        runs on the mosaic) with relaxed thresholds, keeping only new
        bases not already accounted for. Bases `results` already has are
        never reconsidered or overwritten.

        Args:
            corrected_photos: All undistorted/color-corrected photos from
                this mission, cached as (pose, image, gsd_m_per_px,
                camera_altitude_m) tuples.
            results: Bases already confirmed by the first detection pass.
            detect_resolution: (width_px, height_px) of the photos actually
                used for detection (camera.detection_override's resolution
                if set, else camera.resolution).
            templates: Shape-name -> template image map for shape labeling.

        Returns:
            `results` unchanged if there was no shortfall or
            retry_on_shortfall is off; otherwise `results` plus up to
            `shortfall` newly found BaseResults.
        """
        shortfall = self.config.detection.max_bases - len(results)
        if shortfall <= 0 or not self.config.detection.retry_on_shortfall:
            return results

        yasmin.YASMIN_LOG_INFO(
            f'Only {len(results)}/{self.config.detection.max_bases} base(s) found -- '
            f'retrying with relaxed thresholds on the same photos...'
        )
        relaxed_finder = self._build_finder(relaxed=True)
        retry_candidates = [
            c
            for photo in corrected_photos
            for c in self._detect_in_photo(relaxed_finder, photo, detect_resolution, templates)
        ]

        radius_m = self.config.detection.dedup_radius_m
        retry_candidates = _drop_candidates_near(retry_candidates, results, radius_m)

        new_results = deduplicate(retry_candidates, radius_m=radius_m, max_bases=shortfall)
        for result in new_results:
            self._fill_in_incomplete_crop(result, detect_resolution)

        if new_results:
            yasmin.YASMIN_LOG_INFO(f'Retry found {len(new_results)} more base(s).')
        return results + new_results

    def _recover_edge_cut_bases(
        self,
        corrected_photos: List[CorrectedPhoto],
        results: List[BaseResult],
        detect_resolution: Tuple[int, int],
        templates: Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]],
    ) -> List[BaseResult]:
        """Recover a base cut off by more than detection.area_tolerance in
        every overlapping photo: it never passes find()'s area/aspect
        window in any of them, so it never becomes a Detection at all --
        unlike the "slightly cut off" case _fill_in_incomplete_crop()
        already handles from clusters find() DID accept. Second pass with
        finder.find_partial_at_edge() (area/aspect checks skipped for
        contours touching the border -- a partial view can't be expected
        to satisfy them), clustered across photos by proximity same as
        deduplicate(), then composited with merge_base_crop() and
        re-validated with the NORMAL strict find() on the composite --
        only a cluster whose reconstruction looks like a real base once
        merged survives, so a stray reflection at a frame edge can't turn
        into a false positive on its own.

        Args:
            corrected_photos: All undistorted/color-corrected photos from
                this mission, cached as (pose, image, gsd_m_per_px,
                camera_altitude_m) tuples.
            results: Bases already confirmed by earlier passes.
            detect_resolution: (width_px, height_px) of the photos actually
                used for detection.
            templates: Shape-name -> template image map for shape labeling.

        Returns:
            `results` unchanged if there was no shortfall, retry_on_shortfall
            is off, or detection.method == "ia"; otherwise `results` plus up
            to `shortfall` newly recovered BaseResults reconstructed from
            heavily edge-cut clusters.
        """
        shortfall = self.config.detection.max_bases - len(results)
        if shortfall <= 0 or not self.config.detection.retry_on_shortfall:
            return results
        if self.config.detection.method == 'ia':
            return results  # AIBaseFinder.find_partial_at_edge() is a no-op, nothing to do

        finder = self._build_finder()  # normal thresholds, not relaxed
        radius_m = self.config.detection.dedup_radius_m
        partial_candidates: List[BaseCandidate] = []
        for pose, img, gsd, camera_altitude in corrected_photos:
            for det in finder.find_partial_at_edge(img, gsd):
                local_x, local_y = pixel_to_local(
                    det.pixel_center[0],
                    det.pixel_center[1],
                    detect_resolution,
                    gsd,
                    camera_altitude,
                    pose,
                    camera_yaw_offset_deg=self.config.camera.mount.yaw_offset_deg,
                    mount_forward_m=self.config.camera.mount.forward_m,
                    mount_right_m=self.config.camera.mount.right_m,
                )
                partial_candidates.append(
                    BaseCandidate(
                        local_x=local_x,
                        local_y=local_y,
                        detection=det,
                        sharpness=0.0,  # unused: the crop comes from re-validation below, not this
                        pose=pose,
                        gsd_m_per_px=gsd,
                        camera_altitude_m=camera_altitude,
                        source_img=img,
                    )
                )

        partial_candidates = _drop_candidates_near(partial_candidates, results, radius_m)
        clusters = deduplicate(
            partial_candidates, radius_m=radius_m, max_bases=len(partial_candidates)
        )

        window_m = self.config.detection.base_size_m * (1 + _MERGE_MARGIN_FRAC)
        recovered: List[BaseResult] = []
        for cluster_result in clusters:
            if len(recovered) >= shortfall:
                break
            matching = cluster_result.cluster
            merged = merge_base_crop(
                [(c.pose, c.source_img, c.gsd_m_per_px, c.camera_altitude_m) for c in matching],
                base_local_x=cluster_result.local_x,
                base_local_y=cluster_result.local_y,
                base_size_m=self.config.detection.base_size_m,
                resolution=detect_resolution,
                camera_yaw_offset_deg=self.config.camera.mount.yaw_offset_deg,
                mount_forward_m=self.config.camera.mount.forward_m,
                mount_right_m=self.config.camera.mount.right_m,
                margin_frac=_MERGE_MARGIN_FRAC,
            )
            if merged is None:
                continue

            canvas_gsd = min(c.gsd_m_per_px for c in matching)
            revalidated = find_base_squares(
                merged,
                expected_side_px=self.config.detection.base_size_m / canvas_gsd,
                area_tolerance=self.config.detection.area_tolerance,
                white_threshold=self.config.detection.white_threshold,
            )
            if not revalidated:
                continue  # doesn't actually look like a base once reconstructed -- discard

            best = revalidated[0]
            local_x, local_y = canvas_to_local(
                best.pixel_center[0], best.pixel_center[1],
                cluster_result.local_x, cluster_result.local_y,
                window_m, window_m, canvas_gsd,
            )
            shape_label = best.shape_label or match_shape(
                best.shape_contour, best.digit_contour, templates
            )
            recovered.append(
                BaseResult(
                    local_x=local_x,
                    local_y=local_y,
                    crop=best.crop,
                    shape_label=shape_label,
                    num_photos=len(matching),
                )
            )

        if recovered:
            yasmin.YASMIN_LOG_INFO(
                f'Recovered {len(recovered)} heavily cut-off base(s) by merging partial views.'
            )
        return results + recovered

    def execute(self, blackboard: Blackboard) -> str:
        """Run the full vision pipeline (undistort -> color correction ->
        base detection -> pixel->local projection -> dedup, plus the
        shortfall retry and edge-cut recovery passes) over every captured
        photo.

        Args:
            blackboard: Shared mission state. Reads 'captures' (list of
                (CapturePose, raw image) tuples from CaptureWaypoint).
                Writes 'base_results' (list of BaseResult) on success.

        Returns:
            SUCCEED once detection (including retry/recovery passes)
            completes, even with zero bases found; ABORT if an exception/
            KeyboardInterrupt occurs anywhere in the pipeline.
        """
        yasmin.YASMIN_LOG_INFO('DETECTING BASES...')

        try:
            captures = blackboard.get('captures')

            camera_matrix, dist_coeffs = load_calibration(self.config.calibration)

            finder = self._build_finder()
            templates: Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]] = {}
            if self.config.detection.method != 'ia':
                templates_dir = self.config.detection.templates_dir or default_templates_dir()
                templates = load_shape_templates(templates_dir)

            color_cfg = self.config.calibration.color_correction
            candidates: List[BaseCandidate] = []
            corrected_photos: List[CorrectedPhoto] = []

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
                # up_m = camera height above the drone's altitude reference
                # point (rangefinder/EKF), so it adds to altitude_m here.
                camera_altitude = pose.altitude_m + self.config.camera.mount.up_m
                gsd = compute_gsd(camera_altitude, detect_hfov_deg, detect_resolution)

                img = undistort(raw_img, camera_matrix, dist_coeffs)
                if color_cfg.enabled:
                    img = correct_color(
                        img,
                        gray_world_white_balance=color_cfg.gray_world_white_balance,
                        gamma=color_cfg.gamma,
                    )

                photo = (pose, img, gsd, camera_altitude)
                corrected_photos.append(photo)
                candidates.extend(
                    self._detect_in_photo(finder, photo, detect_resolution, templates)
                )

            results = deduplicate(
                candidates,
                radius_m=self.config.detection.dedup_radius_m,
                max_bases=self.config.detection.max_bases,
            )
            for result in results:
                self._fill_in_incomplete_crop(result, detect_resolution)

            results = self._retry_shortfall(
                corrected_photos, results, detect_resolution, templates
            )
            results = self._recover_edge_cut_bases(
                corrected_photos, results, detect_resolution, templates
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


def _demo() -> None:
    # ponytail self-check: _drop_candidates_near() keeps a genuinely new
    # candidate but drops one that's just a weaker re-detection of a base
    # already in `results`, no ROS/State/real detection needed.
    dummy_detection = Detection(
        pixel_center=(0.0, 0.0),
        contour=np.zeros((0, 2)),
        crop=np.zeros((1, 1, 3), dtype=np.uint8),
        fully_in_frame=True,
    )
    existing = [
        BaseResult(
            local_x=1.0, local_y=1.0, crop=dummy_detection.crop, shape_label=None, num_photos=3
        )
    ]
    near = BaseCandidate(local_x=1.1, local_y=1.05, detection=dummy_detection, sharpness=1.0)
    far = BaseCandidate(local_x=-2.0, local_y=-2.0, detection=dummy_detection, sharpness=1.0)

    kept = _drop_candidates_near([near, far], existing, radius_m=0.5)
    assert kept == [far], 'should drop the re-detection near an existing base, keep the new one'
    print('detect_bases self-check OK')


if __name__ == '__main__':
    _demo()
