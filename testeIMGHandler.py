import math

import nectar
from nectar.vision import ImageHandler, OpenCVConfig, CameraFactory
import cv2

from nectar.vision import LineDetector, RotatedRect, ColorSpace

nectar.init()

detector = LineDetector(color="red", estimation_method=RotatedRect, color_space=ColorSpace.LAB)

handler = ImageHandler(
    image_source="webcam",
    config=OpenCVConfig(width=1280, height=720),
    image_processing_callback=lambda frame: detector.detect_line(frame),
    show_result="Camera View",
)

handler.run()
nectar.spin()

if cv2.waitKey(1) & 0xFF == ord('q'):
    nectar.shutdown()