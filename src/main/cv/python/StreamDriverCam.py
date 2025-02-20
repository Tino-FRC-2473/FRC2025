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

print("Running CameraServer")
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

camera1.setFPS(10)
camera2.setFPS(10)

table1 = inst.getTable(f"CameraPublisher/{name1}")
table2 = inst.getTable(f"CameraPublisher/{name2}")

cam1publisher = table1.getStringArrayTopic("streams").publish()
cam2publisher = table2.getStringArrayTopic("streams").publish()

print("Initialization complete")
while True:
    cam1publisher.set(["mjpg:http://10.24.73.105:1181?action=stream"])
    cam2publisher.set(["mjpg:http://10.24.73.105:1182?action=stream"])