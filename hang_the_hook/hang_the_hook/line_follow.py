import math
from nectar.vision import LineDetector, RotatedRect, ColorSpace

detector = LineDetector(color="blue", estimation_method=RotatedRect, color_space=ColorSpace.HSV)

def segue_linha(frame):
    result, mask, cx, cy, angle, w, h = detector.detect_line(frame, draw=True)
    if math.isnan(cx):
        return None, None, None
    print(f"Line at ({cx:.0f}, {cy:.0f}), angle {angle:.1f}")
    return cx, cy, angle