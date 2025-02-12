import time
import cv2
from cscore import CameraServer, VideoSource
import ntcore
import numpy
from config import *
from argparse import ArgumentParser
from visionInput import VisionInput, find_camera_index

if not ON_RPI:
    print("This file can only be run on the raspberry pi")
    exit()

CameraServer.enableLogging()

usb1, name1 = DRIVER_CAM_1
usb2, name2 = DRIVER_CAM_2
index1 = find_camera_index(usb1)
index2 = find_camera_index(usb2)

NETWORK_IDENTITY = "python-drivercam"
inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4(NETWORK_IDENTITY)
inst.setServerTeam(NETWORKTABLES_TEAM)

camera1 = CameraServer.startAutomaticCapture(name1, index1)
camera2 = CameraServer.startAutomaticCapture(name2, index2)

camera1.setResolution(DRIVER_CAM_RES_X, DRIVER_CAM_RES_Y)
camera2.setResolution(DRIVER_CAM_RES_X, DRIVER_CAM_RES_Y)
camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
camera1.setCompression(0)
camera2.setCompression(0)

CameraServer.waitForever()
