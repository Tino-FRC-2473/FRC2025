import cv2
from apriltag import AprilTag
from visionInput import VisionInput
from config import *

RES = (1280, 720)

tag_module = AprilTag()



while True:
    frame = input.getFrame()
    cv2.imshow("frame", frame)
    print("frame size", frame.shape)
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
    #print(tagData)
    
    #cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

