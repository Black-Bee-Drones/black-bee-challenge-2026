import cv2
from nectar.vision import ColorDetector, ColorSpace

detector = ColorDetector(mode="preset", color="yellow", color_space=ColorSpace.HSV)

def detecta_checkpoint(frame):
    detector.filterColor(frame)
    contours, _ = cv2.findContours(detector.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    passou = any(cv2.contourArea(c) > 500 for c in contours)
    return passou, detector.mask ##passou eh bool


##ARRUMAR CHECKPOINT, COR BUGADA!!!!!!!!!!!!!!!!!!!!!!!!