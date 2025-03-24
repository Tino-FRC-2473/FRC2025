import cv2
from apriltag import AprilTag
from visionInput import VisionInput
from config import *

RES = (1280, 720)

tag_module = AprilTag("bw_cam_2v5")
input = VisionInput(AT_FOV, AT_INPUT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, 1)



while True:
    frame = input.getFrame()
    cv2.imshow("frame", frame)
    print("frame size", frame.shape)
    #annotated_frame = frame.copy()
    tagData = tag_module.distance_to_tag(frame, annotated_frame, ARUCO_LENGTH_METERS)
    #print(tagData[3])
    
    #cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break