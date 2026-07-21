import math
from nectar.vision import LineDetector, RotatedRect, ColorSpace

detector = LineDetector(color="blue", estimation_method=RotatedRect, color_space=ColorSpace.HSV)

def segue_linha(frame):
    result, mask, cx, cy, angle, w, h = detector.detect_line(frame, draw=True)
    print("\033[?25l", end="", flush=True)                    # Esconde o cursor
    print(f"\033[{5}F\033[J", end="", flush=True)             # Apaga os prints anteriores do terminal
    if math.isnan(cx):
        return None, None, None
    print(f"Line at ({cx:.0f}, {cy:.0f}), angle {angle:.1f}") # Print line log
    return cx, cy, angle