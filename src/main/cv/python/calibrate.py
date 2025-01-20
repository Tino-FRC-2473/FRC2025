from config import CALIB_RES, CALIB_DIR, ARUCO_LENGTH_METERS, CALIB_WIDTH, CALIB_HEIGHT, CALIB_FILE_NAME
from apriltag import AprilTag

tag_module = AprilTag()

#callibrate based on images
tag_module.calibrate_bw_cam(CALIB_RES, CALIB_DIR, ARUCO_LENGTH_METERS, CALIB_WIDTH, CALIB_HEIGHT, CALIB_FILE_NAME, visualize=False)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry
