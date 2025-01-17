from config import *
from visionInput import VisionInput
from apriltag import AprilTag
import time
import cv2
import traceback

if ON_RPI:
    import ntcore
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("python")
    inst.setServerTeam(2473)

    table = inst.getTable("datatable")
    framePub = table.getDoubleTopic("fps_incremented_value").publish()
    tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
    outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()


FOV = (50.28, 29.16)
RES = (640 , 380)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
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
        # pose_list = [4000 for _ in range(NUM_TAGS * 6)]
        # for key, value in tagData.items():
        #     print("in for loop. key", key, " value: ", value)
        #     pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
        #     print("detected pose_list", pose_list)
        
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
        # print("not pose list:", pose_list)

    if ON_RPI:
        table = inst.getTable("datatable")
        tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
        tagDataPub.set(pose_list)
    print('Loop time: ' + str(time.time()-p))
