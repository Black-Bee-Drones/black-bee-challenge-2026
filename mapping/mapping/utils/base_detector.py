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
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


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


@dataclass
class BaseResult:
    local_x: float
    local_y: float
    crop: np.ndarray
    shape_label: Optional[str]
    num_photos: int


def find_base_squares(
    img: np.ndarray,
    expected_side_px: float,
    area_tolerance: float = 0.35,
    white_threshold: int = 200,
) -> List[Detection]:
    """Detect candidate base squares: bright quadrilaterals of the
    expected pixel size, with a dark shape/number drawn inside.
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

    h, w = img.shape[:2]
    detections = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if not (min_area <= area <= max_area):
            continue

        (cx, cy), (rw, rh), _ = cv2.minAreaRect(contour)
        if rw <= 0 or rh <= 0:
            continue
        aspect = rw / rh if rw > rh else rh / rw
        if aspect > 1.3:  # not square-ish enough
            continue

        x, y, bw, bh = cv2.boundingRect(contour)
        fully_in_frame = x > 0 and y > 0 and (x + bw) < w and (y + bh) < h

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


def deduplicate(
    candidates: Sequence[BaseCandidate], radius_m: float, max_bases: int
) -> List[BaseResult]:
    """Cluster detections of the same physical base (seen in multiple
    overlapping photos) by proximity in local arena coordinates.

    Clusters are ranked by how many photos confirm them (more confirming
    photos = more likely a real base, not a false positive), and only the
    top `max_bases` clusters are kept.
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

        results.append(
            BaseResult(
                local_x=sum(c.local_x for c in cluster) / len(cluster),
                local_y=sum(c.local_y for c in cluster) / len(cluster),
                crop=best.detection.crop,
                shape_label=shape_label,
                num_photos=len(cluster),
            )
        )

    return results
