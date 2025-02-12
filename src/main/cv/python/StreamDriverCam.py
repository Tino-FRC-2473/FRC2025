import time
import cv2
from cscore import CameraServer
import ntcore
import numpy
from config import *
from argparse import ArgumentParser
from visionInput import VisionInput, find_camera_index

if USE_CLI_ARGUMENTS:
    parser = ArgumentParser("streamdrivercam.py", description="2473 Driver Camera Code")
    id_group = parser.add_mutually_exclusive_group(required=True)
    id_group.add_argument("-i", "--index", help="Manual USB index (when not on raspberry pi)")
    id_group.add_argument("-u", "--usb-id", help="USB bus ID (when on raspberry pi)")
    parser.add_argument("port", help="Port to stream the camera on")
    parser.add_argument("cam_name", help="Camera name to show on Shuffleboard")
    args = parser.parse_args()
    if args.index:
        index = int(args.index)
    else:
        usb_id = args.usb_id
    cam_name = args.cam_name
    port = args.port
else:
    index = DRIVER_CAM_INDEX
    usb_id = DRIVER_CAM_USB_ID
    cam_name = DRIVER_CAM_NAME
    port = DRIVER_CAM_LISTEN_PORT

CameraServer.enableLogging()

if ON_RPI:
    NETWORK_IDENTITY = "python-drivercam"
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4(NETWORK_IDENTITY)
    inst.setServerTeam(NETWORKTABLES_TEAM)

    camera = CameraServer.startAutomaticCapture(name=cam_name, dev=find_camera_index(usb_id))
else:
    camera = CameraServer.startAutomaticCapture(name=cam_name, dev=index)


camera.setResolution(DRIVER_CAM_RES_X, DRIVER_CAM_RES_Y)

cv_sink = CameraServer.getVideo()

output_stream = CameraServer.putVideo(cam_name, DRIVER_CAM_RES_X, DRIVER_CAM_RES_Y)

frame = numpy.zeros((DRIVER_CAM_RES_X, DRIVER_CAM_RES_Y, 3), dtype=numpy.uint8)

print("Camera is running")

while True:
    time_stamp, frame = cv_sink.grabFrame(frame)
    if time_stamp == 0:
        print(f"{cam_name} Error: {cv_sink.getError()} ")
        output_stream.notifyError(cv_sink.getError())
        continue

    output_stream.putFrame(frame)

    time.sleep(0.02)