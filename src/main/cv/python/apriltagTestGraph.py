import cv2
from apriltag import AprilTag
from visionInput import VisionInput
from config import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


RES = (1280, 720)

tag_module = AprilTag()
input = VisionInput(AT_FOV, AT_INPUT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, 0)

fig, ax = plt.subplots()

x = []
y = []
line, = ax.plot(x, y)
i = 0
angle = 0

def animate(i):
    global angle
    x.append(i)
    y.append(angle)

    line.set_data(x, y)

    ax.relim()
    ax.autoscale_view()

ani = animation.FuncAnimation(fig, animate, interval=100) 
plt.show(block=False)

while True:
    frame = input.getFrame()
    cv2.imshow("frame", frame)
    print("frame size", frame.shape)
    annotated_frame = frame.copy()
    tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
    if(len(tagData) > 1): 
        angle = tagData[9]
        i += 1

    #print(tagData)
    
    #cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

plt.show()

