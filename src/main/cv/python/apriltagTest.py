import cv2
from apriltag import AprilTag
from visionInput import VisionInput
from config import *
import numpy as np
RES = (1280, 720)

tag_module = AprilTag("bw_cam_4v1")
input = VisionInput(AT_FOV, AT_INPUT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, 0)

while True:
    frame = input.getFrame()
    cv2.imshow("frame", frame)
    # print("frame size", frame.shape)
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)

    #code to debug distance to tag function
    #tags = [10, 0, 0, 0, 3, 3, 3, 9, 9, 9, 11, 0, 0, 0, 2, 2, 2, 9, 9, 9, 12, 0, 0, 0, 1, 1, 1, 9, 9, 9]
    #print(tag_module.sort_tags_distance(tags))

    #print(tagData)
    
    #cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    