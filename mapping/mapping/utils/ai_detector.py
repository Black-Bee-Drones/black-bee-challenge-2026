"""AI-based base detection via a YOLO model (Ultralytics) trained on the
same markers the OpenCV pipeline in base_detector.py looks for by
threshold+contour: a shape outline (Hexagon/Star/Triangle) with a number
(3/4/5) drawn inside it. The model has one class per shape and one per
digit -- not a single "base" class -- so a physical base shows up as two
overlapping detections that must be paired back together here.

Selected via detection.method: "ia" in config.yml (default remains
"opencv"). Requires the `ultralytics` pip package, only imported lazily so
the OpenCV-only path keeps working without it installed.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from mapping.utils.base_detector import Detection

# Model class names -> the Portuguese shape prefixes match_shape() already
# uses (Simulation/Base_Images/{estrela,hexagono,triangulo}{3,4,5}.png),
# so shape_label reads the same regardless of which detector produced it.
_SHAPE_NAMES = {'Hexagon': 'hexagono', 'Star': 'estrela', 'Triangle': 'triangulo'}
_NUMBER_NAMES = {'3', '4', '5'}

BoxDetection = Tuple[str, Tuple[float, float, float, float]]  # (class_name, (x0, y0, x1, y1))


def load_model(model_path: str):
    """Load the Ultralytics YOLO model from a .pt checkpoint path.

    Args:
        model_path: Filesystem path to the trained YOLO `.pt` checkpoint.

    Returns:
        The loaded `ultralytics.YOLO` model, ready for `.predict()`.
    """
    from ultralytics import YOLO
    return YOLO(model_path)


PairedDetection = Tuple[str, Tuple[float, float, float, float], Optional[str]]


def _pair_shapes_and_numbers(boxes: Sequence[BoxDetection]) -> List[PairedDetection]:
    """Pair each shape box with the nearest number box whose center falls
    inside it (the number is drawn inside the shape's outline, so a center
    distance under half the shape's side is a safe match). Returns
    (shape_name, shape_box, matched_number_or_None) per shape detected --
    unmatched number boxes are dropped, since without a shape there's no
    base square to anchor the crop/pixel_center on.

    Args:
        boxes: Raw (class_name, (x0, y0, x1, y1)) detections from the YOLO
            model, in pixel coordinates, mixing shape and digit classes.

    Returns:
        One (shape_name, shape_box, matched_number_or_None) tuple per shape
        box detected.
    """
    shapes = [(name, box) for name, box in boxes if name in _SHAPE_NAMES]
    numbers = [(name, box) for name, box in boxes if name in _NUMBER_NAMES]

    paired = []
    for shape_name, (x0, y0, x1, y1) in shapes:
        scx, scy = (x0 + x1) / 2, (y0 + y1) / 2
        side = max(x1 - x0, y1 - y0)

        nearest_number, nearest_dist = None, None
        for num_name, (nx0, ny0, nx1, ny1) in numbers:
            ncx, ncy = (nx0 + nx1) / 2, (ny0 + ny1) / 2
            dist = math.hypot(ncx - scx, ncy - scy)
            if dist <= side / 2 and (nearest_dist is None or dist < nearest_dist):
                nearest_number, nearest_dist = num_name, dist

        paired.append((shape_name, (x0, y0, x1, y1), nearest_number))

    return paired


def find_base_squares_ai(
    img: np.ndarray, model, confidence: float = 0.5, pad_frac: float = 0.15
) -> List[Detection]:
    """Detect candidate base squares with the YOLO model, returning
    Detection objects with shape_label already filled in (bypassing
    match_shape() -- see Detection.shape_label).

    Args:
        img: BGR image to run detection on.
        model: Loaded `ultralytics.YOLO` model (see `load_model`).
        confidence: Minimum detection confidence passed to the model.
        pad_frac: Padding added around each detected box, as a fraction of
            the box's longest side, when cropping the proof photo.

    Returns:
        One `Detection` per shape box found, with `shape_label` set.
    """
    h, w = img.shape[:2]
    result = model.predict(img, conf=confidence, verbose=False)[0]
    names = result.names

    boxes: List[BoxDetection] = [
        (names[int(box.cls[0])], tuple(box.xyxy[0].tolist()))
        for box in result.boxes
    ]

    detections = []
    for shape_name, (x0, y0, x1, y1), number in _pair_shapes_and_numbers(boxes):
        label = _SHAPE_NAMES[shape_name] + number if number else _SHAPE_NAMES[shape_name]
        side = max(x1 - x0, y1 - y0)
        pad = side * pad_frac

        cx0, cy0 = max(0, int(x0 - pad)), max(0, int(y0 - pad))
        cx1, cy1 = min(w, int(x1 + pad)), min(h, int(y1 + pad))
        fully_in_frame = x0 > 0 and y0 > 0 and x1 < w and y1 < h

        detections.append(
            Detection(
                pixel_center=((x0 + x1) / 2, (y0 + y1) / 2),
                contour=np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32),
                crop=img[cy0:cy1, cx0:cx1].copy(),
                fully_in_frame=fully_in_frame,
                shape_label=label,
            )
        )

    return detections


def _demo():
    # ponytail self-check: pairing logic, no model/image required.
    boxes: List[BoxDetection] = [
        ('Hexagon', (100, 100, 200, 200)),  # side=100, center=(150,150)
        ('3', (140, 140, 160, 160)),  # center=(150,150), dist=0 -> pairs
        ('Star', (300, 300, 340, 340)),  # side=40, center=(320,320)
        ('5', (400, 400, 420, 420)),  # far away -> no pair
    ]
    paired = _pair_shapes_and_numbers(boxes)
    by_shape = {name: number for name, _, number in paired}
    assert by_shape['Hexagon'] == '3', by_shape
    assert by_shape['Star'] is None, by_shape
    print('ai_detector self-check OK')


if __name__ == '__main__':
    _demo()
