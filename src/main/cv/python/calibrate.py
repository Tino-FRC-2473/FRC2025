import cv2
import numpy as np
from apriltag import AprilTag
from visionInput import VisionInput
import time

RES = (640, 480)

tag_module = AprilTag()
#change this depending on which directory images for camera callibration are in
CALIB_DIR = 'bw-cam1-images'
CALIB_WIDTH = 7
CALIB_HEIGHT = 7
ARUCO_LENGTH_METERS = 0.025

#callibrate based on images
tag_module.calibrate_bw_cam(RES, CALIB_DIR, ARUCO_LENGTH_METERS, CALIB_WIDTH, CALIB_HEIGHT, "bw_cam_1", visualize=False)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry




