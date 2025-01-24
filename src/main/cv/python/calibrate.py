from config import *
import argparse
from apriltag import AprilTag
import sys
'''
# https://stackoverflow.com/a/4042861
class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\n")
        self.print_help()
        sys.exit(2)

parser = MyParser("calibrate", description="Calibrate AprilTag camera")
parser.add_argument("cam_name", help="Name of camera/file to be saved")
parser.add_argument("-c", "--color", help="Colored camera", action="store_true")

if sys.argv == 1:
    parser.print_help()
    parser.exit()

args = parser.parse_args()
'''

CALIB_RES = (1280, 720)
CALIB_DIR = 'calibration_images'
CALIB_FILE_NAME = "bw_cam_1v3"
CALIB_WIDTH = 6
CALIB_HEIGHT = 4
ARUCO_LENGTH_METERS = 0.03
BW_CAMERA = True

tag_module = AprilTag()

#callibrate based on images
tag_module.calibrate(CALIB_RES, CALIB_DIR, ARUCO_LENGTH_METERS, CALIB_WIDTH, CALIB_HEIGHT,CALIB_FILE_NAME, BW_CAMERA, visualize=False)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry
