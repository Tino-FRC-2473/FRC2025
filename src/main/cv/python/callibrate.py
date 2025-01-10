import cv2
import numpy as np
from apriltag import AprilTag
from visionInput import VisionInput
import time

RES = (640, 480)

tag_module = AprilTag()
#change this depending on which directory images for camera callibration are in
CALIB_DIR = 'bw-cam1-images'
CALIB_SIZE_METERS = 0.015
CALIB_WIDTH = 9
CALIB_HEIGHT = 9


#callibrate based on images
tag_module.calibrate(RES, CALIB_DIR, CALIB_SIZE_METERS, CALIB_WIDTH, CALIB_HEIGHT,True, visualize=False)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry

FOV = (50.28, 29.16)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
TAG_LENGTH_METERS = 0.165


