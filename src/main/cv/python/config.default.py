## Copy to config.py

ON_RPI = False

DRIVER_CAM_LISTEN_PORT = 1181
DRIVER_CAM_LISTEN_IP = '0.0.0.0' # 0.0.0.0 means it will listen on all IPs
DRIVER_CAM_USB_ID = 'usb-xhci-hcd.0-2' # for when on RPi
DRIVER_CAM_INDEX = 0 # for when not on RPi

AT_CAM_USB_ID = 'usb-xhci-hcd.0-1' # for when on RPi
AT_CAM_INDEX = 0 # for when not on RPi
AT_FOV = (50.28, 29.16)
AT_RES = (640 , 380)
AT_CAM_HEIGHT = 0.4
AT_CAM_ANGLE = -15

NETWORKTABLES_TEAM = 2473

#change this depending on which directory images for camera callibration are in
CALIB_RES = (640, 480)
CALIB_DIR = 'bw-cam1-images'
CALIB_FILE_NAME = "bw_cam_1"
CALIB_WIDTH = 7
CALIB_HEIGHT = 7
ARUCO_LENGTH_METERS = 0.025
