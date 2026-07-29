import math
import nectar
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision import LineDetector, RotatedRect, ColorSpace
from nectar.control import PIDController, AltitudeSource, MavrosDrone

import cv2

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime

from followlineSM.constants import (
    CENTER_VARIATION
)

class SetupLineDetection(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.blackboard = Blackboard()
        self.linedetector = None
        self.hosedetector = None
        self.handler = None
        self.node = YasminNode.get_instance()

    def execute(self, Blackboard: Blackboard):
        try:
            # Initialize the line linedetector with the desired color and color space
            self.linedetector = LineDetector(color="blue", estimation_method=RotatedRect, color_space=ColorSpace.HSV)
            self.hosedetector = LineDetector(color="red", estimation_method=RotatedRect, color_space=ColorSpace.HSV)

            # Set up the image handler with the webcam as the source
            self.handler = ImageHandler(
                image_source="webcam",
                config=OpenCVConfig(width=1280, height=720),
                image_processing_callback=lambda frame: self.linedetector.detect_line(frame),
                show_result="Camera View",
            )

            # Store the linedetector and handler in the Blackboard for later use
            Blackboard["line_detector"] = self.linedetector
            Blackboard["hose_detector"] = self.hosedetector
            Blackboard["image_handler"] = self.handler

            return SUCCEED
        except Exception as e:
            print(f"Setup failed: {e}")
            return ABORT

class SearchBlueLine(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.blackboard = Blackboard()
        self.node = YasminNode.get_instance()

    def execute(self, Blackboard: Blackboard):
        try:
            # Retrieve the line linedetector and image handler from the Blackboard
            linedetector = Blackboard["line_linedetector"]
            handler = Blackboard["image_handler"]
            counterblue = 0
            oldcxb = 0
            oldcyb = 0

            if not linedetector or not handler:
                print("One or more detectors or image handler not initialized.")
                return ABORT

            # Start the image handler to process frames and detect lines
            handler.start()

            # Main loop for line following
            while True:
                frame = handler.take_photo()
                resultb, _, cxb, cyb, angleb, wb, hb = linedetector.detect_line(frame, draw=True)

                #Skips if no line detected
                if not resultb or cxb is None or cyb is None:
                    continue

                distanceb = math.dist((cxb,cyb),(oldcxb,oldcyb))

                if (distanceb < CENTER_VARIATION):
                    counterblue = counterblue + 1

                if counterblue == 5:
                    counterblue = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite("../images/{now}.png", resultb)
                    Blackboard["angle_blue"] = angleb
                    Blackboard["width_blue"] = wb
                    Blackboard["height_blue"] = hb
                    break

                oldcxb = cxb
                oldcyb = cyb

            return SUCCEED
        except Exception as e:
            print(f"Blue line searching failed: {e}")
            return ABORT
        finally:
            handler.stop()
