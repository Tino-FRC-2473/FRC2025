ON_RPI = False # Enable if on Raspberry Pi. Enabled NetworkTables stuff and disables imshow
USE_CLI_ARGUMENTS = False # Require CLI arguments when running main.py and StreamDriverCam.py. Used for multiple cameras.

DRIVER_CAM_LISTEN_PORT = 1181
DRIVER_CAM_1 = ('usb-xhci-hcd.0-2', 'front') # ( usb_id, name )
DRIVER_CAM_2 = ('usb-xhci-hcd.1-2', 'back') # ( usb_id, name )
DRIVER_CAM_RES_X = 320 # Horizontal resolution of the driver cameras
DRIVER_CAM_RES_Y = 240 # Vertical resolution of the driver cameras

if not USE_CLI_ARGUMENTS:
    AT_CAM_USB_ID = 'usb-xhci-hcd.0-1' # for when on RPi
    AT_CAM_INDEX = 0 # for when not on RPi
    AT_CAM_NAME = "bw_cam" # used for npy files
AT_NPY_DIR = "calibration_data" # relative to python files
AT_FOV = (50.28, 29.16) # degrees
AT_INPUT_RES = (1280, 720) # If this resolution is not supported by the camera, it will use the camera's default res
AT_RESIZED_RES = (320, 240)
AT_CAM_HEIGHT = 0.4
AT_Z_OFFSET = 0
AT_X_OFFSET = 0
AT_CAM_ANGLE = -15 # degrees
ARUCO_LENGTH_METERS = 0.165

NETWORKTABLES_TEAM = 2473

#change this depending on which directory images for camera callibration are in
CALIB_RES = (1280, 720)
CALIB_INPUT_DIR = 'calibration_images' # relative to python files
CALIB_OUTPUT_DIR = AT_NPY_DIR # relative to python files
CALIB_WIDTH = 6
CALIB_HEIGHT = 4
SQUARE_LENGTH_METERS = 0.017

DATA_COLLECTION_ENABLED = True
DATA_COLLECTION_DIR = 'data_collection' # relative to python files
DATA_COLLECTION_FPS = 10.0 # since frame rate is not constant, the data collection video is not in real time

# USB ids on a Pi 5:
# | Position     | Id                 |
# | ------------ | ------------------ |
# | Top-Left     | `usb-xhci-hcd.1-1` |
# | Top-Right    | `usb-xhci-hcd.0-2` |
# | Bottom-Right | `usb-xhci-hcd.1-2` |
# | Bottom-Left  | `usb-xhci-hcd.0-1` |
