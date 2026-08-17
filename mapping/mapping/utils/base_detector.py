"""Base detection: finds the white 80x80cm squares (hexagon/triangle/star
markers) on the arena floor, and deduplicates detections coming from
overlapping photos.

Per the competition rules, classifying the shape/number drawn on each
base is NOT required for scoring -- it exists only so a human can verify
that each submitted photo shows a distinct base. Shape matching against
the reference templates in Simulation/Base_Images is used here only as
an optional confidence signal while deduplicating, never for scoring.
"""

import glob
import math
import os
import re
from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from mapping.utils.geo_projection import CapturePose


@dataclass
class Detection:
    pixel_center: Tuple[float, float]
    contour: np.ndarray
    crop: np.ndarray  # cropped proof photo of just the base
    fully_in_frame: bool  # whether the whole base square is within the image bounds
    # Contours of the dark drawing *inside* the white square -- what
    # actually needs to be compared against the templates (`contour` above
    # is the outer square boundary, which every base shares, so matching
    # against it can never tell hexagono3 from triangulo5). `shape_contour`
    # is the outer polygon (hexagon/triangle/star outline); `digit_contour`
    # is the smaller number drawn inside it.
    shape_contour: Optional[np.ndarray] = None
    digit_contour: Optional[np.ndarray] = None
    # Set directly by the AI detector (ai_detector.py), which classifies
    # shape+number from the model's own class predictions instead of
    # contour matching -- when present, detect_bases.py skips match_shape().
    shape_label: Optional[str] = None


@dataclass
class BaseCandidate:
    local_x: float
    local_y: float
    detection: Detection
    sharpness: float
    shape_label: Optional[str] = None
    weight: float = 1.0  # from centrality_weight(); used by deduplicate()'s weighted average
    # Only set by detect_bases.py when detection.fully_in_frame is False --
    # what merge_base_crop() (utils/mosaic.py) needs to reproject this
    # photo later if the whole cluster turns out to have no fully-in-frame
    # photo at all (base straddling two waypoints' footprints). None for
    # the (usual) fully-in-frame case, so most photos don't pay the memory
    # cost of holding onto their full corrected image.
    pose: Optional[CapturePose] = None
    gsd_m_per_px: Optional[float] = None
    camera_altitude_m: Optional[float] = None
    source_img: Optional[np.ndarray] = None


@dataclass
class BaseResult:
    local_x: float
    local_y: float
    crop: np.ndarray
    shape_label: Optional[str]
    num_photos: int
    # The winning cluster, so the caller can tell whether `crop` came from
    # a single fully-in-frame photo or needs merge_base_crop() -- see
    # states/mission/detect_bases.py.
    cluster: List[BaseCandidate] = field(default_factory=list)


def find_base_squares(
    img: np.ndarray,
    expected_side_px: float,
    area_tolerance: float = 0.35,
    white_threshold: int = 200,
    accept_partial_at_edge: bool = False,
) -> List[Detection]:
    """Detect candidate base squares: bright quadrilaterals of the
    expected pixel size, with a dark shape/number drawn inside.

    accept_partial_at_edge (default False, unchanged normal behavior):
    a contour touching the image border could be a base cut off by the
    frame edge rather than noise -- its visible area/aspect can't be
    expected to match a whole square, so such contours skip the
    area/aspect checks below (just a lenient noise floor instead) when
    this is set. Used by DetectBases._recover_edge_cut_bases() to find
    candidates for merge_base_crop() that the normal area/aspect window
    would otherwise reject before they ever became a Detection -- the
    merged result still has to pass the normal, strict check (this
    function again, with accept_partial_at_edge=False) to become a real
    base, so a stray reflection at a frame edge can't become a false
    positive on its own.

    Args:
        img: BGR (or grayscale) photo to search for base squares.
        expected_side_px: Expected side length of a base square in this
            photo, in pixels, at the flight altitude/GSD.
        area_tolerance: Allowed fractional deviation from the expected
            square area (e.g. 0.35 accepts 65%-135% of expected_area).
        white_threshold: Grayscale value (0-255) above which a pixel counts
            as part of the white base square.
        accept_partial_at_edge: Whether to accept contours touching the
            image border under a lenient noise floor instead of the normal
            area/aspect window (see docstring above).

    Returns:
        One `Detection` per candidate base square found in the photo.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    _, mask = cv2.threshold(gray, white_threshold, 255, cv2.THRESH_BINARY)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    expected_area = expected_side_px**2
    min_area = expected_area * (1 - area_tolerance)
    max_area = expected_area * (1 + area_tolerance)
    partial_min_area = expected_area * 0.05  # noise floor for edge-touching contours

    h, w = img.shape[:2]
    detections = []

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, bw, bh = cv2.boundingRect(contour)
        fully_in_frame = x > 0 and y > 0 and (x + bw) < w and (y + bh) < h

        (cx, cy), (rw, rh), _ = cv2.minAreaRect(contour)

        if accept_partial_at_edge and not fully_in_frame:
            if area < partial_min_area:
                continue
        else:
            if not (min_area <= area <= max_area):
                continue
            if rw <= 0 or rh <= 0:
                continue
            aspect = rw / rh if rw > rh else rh / rw
            if aspect > 1.3:  # not square-ish enough
                continue

        # Dark shape/number drawn inside the white square: inverted threshold
        # within just the square's own bounding box, so the drawing's own
        # contours (not the square's) are what gets compared to the
        # templates. RETR_LIST (not EXTERNAL) because the outline polygon
        # and the digit are nested inside the same white region -- EXTERNAL
        # would discard the digit as a "child" of the polygon.
        inner_mask = cv2.threshold(
            gray[y:y + bh, x:x + bw], white_threshold, 255, cv2.THRESH_BINARY_INV
        )[1]
        inner_mask = cv2.morphologyEx(inner_mask, cv2.MORPH_CLOSE, kernel)
        inner_contours, _ = cv2.findContours(inner_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        shape_contour, digit_contour = _split_outline_and_digit(inner_contours, bw, bh)

        pad = int(max(bw, bh) * 0.15)
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(w, x + bw + pad), min(h, y + bh + pad)

        detections.append(
            Detection(
                pixel_center=(float(cx), float(cy)),
                contour=contour,
                crop=img[y0:y1, x0:x1].copy(),
                fully_in_frame=fully_in_frame,
                shape_contour=shape_contour,
                digit_contour=digit_contour,
            )
        )

    return detections


def _split_outline_and_digit(
    contours: Sequence[np.ndarray], region_w: int, region_h: int, small_frac: float = 0.5
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Split a region's inner contours into (outline, digit): the outline
    polygon (hexagon/triangle/star) spans most of the region, the digit
    drawn inside it is comfortably smaller -- so the largest contour whose
    bounding box is under `small_frac` of the region on both axes is taken
    as the digit, and the single largest overall as the outline.

    Args:
        contours: Candidate contours found inside a base square's bounding box.
        region_w: Width of the region the contours were found in, in pixels.
        region_h: Height of the region the contours were found in, in pixels.
        small_frac: Fraction of `region_w`/`region_h` a contour's bounding
            box must stay under, on both axes, to be considered the digit.

    Returns:
        (outline, digit): the largest contour (the shape outline) and the
        largest contour small enough to be the digit, or None for either
        if `contours` was empty / no contour qualified as the digit.
    """
    if not contours:
        return None, None
    by_area = sorted(contours, key=cv2.contourArea, reverse=True)
    outline = by_area[0]
    digit = next(
        (
            c for c in by_area
            if (lambda bw, bh: bw < region_w * small_frac and bh < region_h * small_frac)(
                *cv2.boundingRect(c)[2:]
            )
        ),
        None,
    )
    return outline, digit


# Canonical base names this project actually places in the arena (see
# Simulation/scripts/random_base_location.py's BASE_MODELS) -- the
# directory also has redundant/duplicate renders under other names (e.g.
# "3.png", "trian3.png" are the same marker as "hexagono3.png"), which
# would just create ambiguous ties if all loaded.
_SHAPE_NAME_RE = re.compile(r'^(estrela|hexagono|triangulo)([345])$')


def load_shape_templates(templates_dir: str) -> Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]]:
    """Load (outline, digit) contour pairs from Simulation/Base_Images for
    shape+number classification (informational only -- the competition
    rules don't require it for scoring).

    Args:
        templates_dir: Directory containing the reference marker PNGs
            (e.g. Simulation/Base_Images), named like "hexagono3.png".

    Returns:
        Mapping of canonical marker name (e.g. "hexagono3") to its
        (outline, digit) contour pair, `digit` possibly None.
    """
    templates = {}
    for path in glob.glob(os.path.join(templates_dir, '*.png')):
        name = os.path.splitext(os.path.basename(path))[0]
        if not _SHAPE_NAME_RE.match(name):
            continue
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        h, w = img.shape[:2]
        outline, digit = _split_outline_and_digit(contours, w, h)
        if outline is None:
            continue
        templates[name] = (outline, digit)
    return templates


def match_shape(
    shape_contour: Optional[np.ndarray],
    digit_contour: Optional[np.ndarray],
    templates: Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]],
) -> Optional[str]:
    """Best-matching template name (e.g. "hexagono3"), combining the outer
    outline's match score with the digit's when both are available.

    Args:
        shape_contour: Detected outline contour (hexagon/triangle/star),
            or None if none was found.
        digit_contour: Detected digit contour, or None if none was found.
        templates: Reference (outline, digit) contour pairs keyed by
            canonical marker name, from `load_shape_templates`.

    Returns:
        The best-matching template name, or None if `templates` is empty
        or `shape_contour` is None.
    """
    if not templates or shape_contour is None:
        return None

    scores = {}
    for name, (t_outline, t_digit) in templates.items():
        score = cv2.matchShapes(shape_contour, t_outline, cv2.CONTOURS_MATCH_I1, 0.0)
        if digit_contour is not None and t_digit is not None:
            score += cv2.matchShapes(digit_contour, t_digit, cv2.CONTOURS_MATCH_I1, 0.0)
        scores[name] = score
    return min(scores, key=scores.get)


def centrality_weight(
    pixel_center: Tuple[float, float], resolution: Tuple[int, int], floor: float = 0.1
) -> float:
    """Weight in [floor, 1.0] for how close a detection is to the image's
    optical center (1.0 = dead center, floor = a corner). pixel_to_local()
    assumes a perfectly nadir camera; any small residual drone tilt makes
    that assumption's error grow with distance from the principal point,
    so weighting a detection's contribution to deduplicate()'s averaged
    position by this pulls the estimate toward the least-distorted
    readings instead of trusting every repeated observation equally.

    # ponytail: this is a statistical proxy for tilt error, not a fix of
    # the root cause -- CapturePose carries no roll/pitch, so
    # pixel_to_local() can't do a real ray-ground intersection. Upgrade
    # path: capture drone attitude per photo (MAVROS EKF) and project with
    # the full rotation instead of the flat nadir assumption; only then
    # would this weighting become unnecessary.

    Args:
        pixel_center: (col, row) pixel position of the detection's center.
        resolution: (width_px, height_px) of the photo.
        floor: Minimum weight returned, for a detection at a corner.

    Returns:
        Weight in [floor, 1.0]; 1.0 at the optical center, `floor` at a corner.
    """
    width_px, height_px = resolution
    half_diag = math.hypot(width_px, height_px) / 2
    dist = math.hypot(pixel_center[0] - width_px / 2, pixel_center[1] - height_px / 2)
    return max(floor, 1.0 - dist / half_diag)


def deduplicate(
    candidates: Sequence[BaseCandidate], radius_m: float, max_bases: int
) -> List[BaseResult]:
    """Cluster detections of the same physical base (seen in multiple
    overlapping photos) by proximity in local arena coordinates.

    Clusters are ranked by how many photos confirm them (more confirming
    photos = more likely a real base, not a false positive), and only the
    top `max_bases` clusters are kept.

    Args:
        candidates: Per-photo base detections with their projected local
            arena position.
        radius_m: Maximum distance, in meters, between two candidates for
            them to be considered the same physical base.
        max_bases: Maximum number of base clusters to return.

    Returns:
        Up to `max_bases` `BaseResult`s, one per detected base, ranked by
        number of confirming photos (most confirmed first).
    """
    remaining = list(candidates)
    clusters: List[List[BaseCandidate]] = []

    while remaining:
        seed = remaining.pop(0)
        cluster = [seed]
        i = 0
        while i < len(remaining):
            c = remaining[i]
            if any(
                math.hypot(c.local_x - m.local_x, c.local_y - m.local_y) <= radius_m
                for m in cluster
            ):
                cluster.append(remaining.pop(i))
            else:
                i += 1
        clusters.append(cluster)

    clusters.sort(key=len, reverse=True)

    results = []
    for cluster in clusters[:max_bases]:
        # Prefer a photo where the base is fully in frame (required for
        # scoring), then the sharpest among those.
        best = max(cluster, key=lambda c: (c.detection.fully_in_frame, c.sharpness))

        # Majority vote across every confirming photo, not just `best`'s
        # single guess -- shape/number matching is noisy per-frame, but the
        # same physical base should get the same label most of the time.
        labels = [c.shape_label for c in cluster if c.shape_label is not None]
        shape_label = Counter(labels).most_common(1)[0][0] if labels else None

        # Weighted average: detections closer to the image's optical center
        # (higher weight, see centrality_weight()) carry less nadir-assumption
        # error, so they should pull the averaged position harder.
        total_weight = sum(c.weight for c in cluster)

        results.append(
            BaseResult(
                local_x=sum(c.local_x * c.weight for c in cluster) / total_weight,
                local_y=sum(c.local_y * c.weight for c in cluster) / total_weight,
                crop=best.detection.crop,
                shape_label=shape_label,
                num_photos=len(cluster),
                cluster=list(cluster),
            )
        )

    return results


def _demo() -> None:
    # ponytail self-check: accept_partial_at_edge lets a heavily cut-off
    # square through (skipping the area/aspect window a partial view
    # can't be expected to satisfy), while leaving the normal in-frame
    # case identical either way.
    img = np.zeros((60, 60, 3), dtype=np.uint8)
    # A 40x40 white square, only its left third (cols 0-14) inside this
    # 60x60 frame -- like a base cut off by >60% at the image edge.
    img[10:50, 0:15] = 255
    expected_side_px = 40.0

    assert find_base_squares(img, expected_side_px, area_tolerance=0.35) == []
    partial = find_base_squares(
        img, expected_side_px, area_tolerance=0.35, accept_partial_at_edge=True
    )
    assert len(partial) == 1 and partial[0].fully_in_frame is False

    full = np.zeros((60, 60, 3), dtype=np.uint8)
    full[10:50, 10:50] = 255
    normal = find_base_squares(full, expected_side_px, area_tolerance=0.35)
    with_flag = find_base_squares(
        full, expected_side_px, area_tolerance=0.35, accept_partial_at_edge=True
    )
    assert len(normal) == 1 and normal[0].fully_in_frame is True
    assert len(with_flag) == 1 and with_flag[0].fully_in_frame is True

    print('base_detector self-check OK')


if __name__ == '__main__':
    _demo()
