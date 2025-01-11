import cv2
import numpy as np
from apriltag import AprilTag
from visionInput import VisionInput
import time

RES = (640, 480)

tag_module = AprilTag()


FOV = (50.28, 29.16)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
TAG_LENGTH_METERS = 0.165

while True:
    frame = input.getFrame()
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_pose(frame, annotated_frame, TAG_LENGTH_METERS)
    print(tagData)
    pose_list = [4000 for _ in range(16 * 6)]
    for key, value in tagData.items():
        pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
    
#    cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

