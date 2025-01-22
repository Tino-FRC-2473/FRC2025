from config import *
import argparse
from apriltag import AprilTag
import sys

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
    exit()

parser.parse_args()

tag_module = AprilTag()

#callibrate based on images
tag_module.calibrate(CALIB_RES, CALIB_DIR, ARUCO_LENGTH_METERS, CALIB_WIDTH, CALIB_HEIGHT, parser.cam_name, parser.color, visualize=False)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry
