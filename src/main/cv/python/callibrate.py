import cv2
import numpy as np
from apriltag import AprilTag
from visionInput import VisionInput
import time

RES = (640, 480)

tag_module = AprilTag()
CALIB_DIR = 'bw_cam_num_1'
CALIB_SIZE_METERS = 0.015
CALIB_WIDTH = 9
CALIB_HEIGHT = 9
tag_module.calibrate(RES, CALIB_DIR, CALIB_SIZE_METERS, CALIB_WIDTH, CALIB_HEIGHT, visualize=False)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry

FOV = (50.28, 29.16)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
TAG_LENGTH_METERS = 0.165

'''
while True:
    frame = input.getFrame()
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_pose(frame, annotated_frame, TAG_LENGTH_METERS)
    cv2.imshow('result', annotated_frame)
    pose_list = [4000 for _ in range(16 * 6)]
    if(tagData != None):
        for key, value in tagData.items():
            pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
    
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
'''
