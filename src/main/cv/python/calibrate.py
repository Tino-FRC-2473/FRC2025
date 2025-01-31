"""
To run calibrate.py:
    python calibrate.py cam_name
Run the following to list additional options (visualize, colored camera, etc.):
    python calibrate.py -h
"""

from config import *
import argparse
from apriltag import calibrate_camera
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
parser.add_argument("-v", "--visualize", help="Show GUI with images", action="store_true")

if sys.argv == 1:
    parser.print_help()
    parser.exit()

args = parser.parse_args()

#callibrate based on images
calibrate_camera(CALIB_RES, CALIB_INPUT_DIR, CALIB_OUTPUT_DIR, ARUCO_LENGTH_METERS, CALIB_WIDTH, CALIB_HEIGHT, args.cam_name, args.color, args.visualize)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry
