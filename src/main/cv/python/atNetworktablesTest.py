from config import *
from visionInput import VisionInput, find_camera_index
from apriltag import AprilTag
import time
import cv2
import traceback

if ON_RPI:
    import ntcore
    NETWORK_IDENTITY = "python"
    TEAM_NUMBER = 2473
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4(NETWORK_IDENTITY)
    inst.setServerTeam(TEAM_NUMBER)

    table = inst.getTable("datatable")
    framePub = table.getDoubleTopic("fps_incremented_value").publish()
    tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
    outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()

if ON_RPI:
    index = find_camera_index(AT_CAM_USB_ID)
else:
    index = AT_CAM_INDEX

input = VisionInput(AT_FOV, AT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, index)

tag_module = AprilTag()
ARUCO_LENGTH_METERS = 0.165
pose_list=[]
NUM_TAGS = 22

while True:
    p = time.time()
    pose_list = [4000 for _ in range(NUM_TAGS * 6)]
    try: 
        frame = input.getFrame()
        print("framesize", frame.shape)
        annotated_frame = frame.copy()
        tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
        annotated_frame = cv2.resize(annotated_frame, (320,240))
        if(tagData is None):
            print("tagData none")
        print("tagData.items:", tagData)
        
        if ON_RPI:
            framePub.set(frame.sum())
            tagDataPub.set(pose_list)
            outputStreamPub.set(annotated_frame.flatten().tolist())

        cv2.imshow('result', annotated_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        time.sleep(0.02)
    except KeyboardInterrupt:
        print("keyboard interrupt")
        input.close()
        break
    except Exception as error:
        print("An exception occurred:", error.__class__)
        traceback.print_exc()

    if ON_RPI:
        table = inst.getTable("datatable")
        tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
        tagDataPub.set(pose_list)
    print('Loop time: ' + str(time.time()-p))
