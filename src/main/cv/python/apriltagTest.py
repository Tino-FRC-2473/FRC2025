import cv2
import numpy as np
from apriltag import AprilTag
from visionInput import VisionInput
import time

RES = (640, 480)

tag_module = AprilTag()


FOV_DEGREES = (50.28, 29.16)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV_DEGREES, RES, CAM_HEIGHT, CAM_ANGLE)
TAG_LENGTH_METERS = 0.165
NUM_TAGS = 22
T
while True:
    frame = input.getFrame()
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_poqqse(frame, annotated_frame, TAG_LENGTH_METERS)
    print(tagData)
    
#    cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

