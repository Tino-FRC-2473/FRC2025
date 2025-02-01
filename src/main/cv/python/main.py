from config import *
from visionInput import VisionInput, find_camera_index
from data_collector import DataCollector
from apriltag import AprilTag
import time
import cv2
import traceback

if ON_RPI:
    import ntcore
    NETWORK_IDENTITY = "python"
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4(NETWORK_IDENTITY)
    inst.setServerTeam(NETWORKTABLES_TEAM)

    table = inst.getTable("datatable")
    framePub = table.getDoubleTopic("fps_incremented_value").publish()
    tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
    outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()

if ON_RPI:
    index = find_camera_index(AT_CAM_USB_ID)
else:
    index = AT_CAM_INDEX

input = VisionInput(AT_FOV, AT_INPUT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, index)

tag_module = AprilTag()
tagData = []

frame_idx = 0

if ON_RPI:
    table = inst.getTable("datatable")

if DATA_COLLECTION_ENABLED:
    # check whether resolution is correct
    initial_frame = input.getFrame()
    data_collector = DataCollector(initial_frame.shape)

while True:
    loop_start = time.time()
    try: 
        initial_frame = input.getFrame()
        annotated_frame = initial_frame.copy()
        tagData = tag_module.estimate_3d_pose(initial_frame, annotated_frame, ARUCO_LENGTH_METERS)
        annotated_frame = cv2.resize(annotated_frame, AT_RESIZED_RES)
        if(tagData is None):
            print("tagData none")
        print("tagData.items:", tagData)

        if DATA_COLLECTION_ENABLED:
            data_collector.write_frame(initial_frame, annotated_frame, tagData)
        
        if ON_RPI:
            #when tagData is none a empty frame will be sent over
            framePub.set(initial_frame.sum())
            tagDataPub.set(tagData)
            outputStreamPub.set(annotated_frame.flatten().tolist())
        else:
            cv2.imshow('result', annotated_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        #time.sleep(0.1)
    except KeyboardInterrupt:
        print("keyboard interrupt")
        framePub.close()
        tagDataPub.close()
        outputStreamPub.close()
        input.close()
        break
    except Exception as error:
        print("An exception occurred:", error.__class__)
        traceback.print_exc()

    frame_idx %= 5
    if ON_RPI and frame_idx == 0:
        tagDataPub = tagDataPub.set()
        tagDataPub.set(tagData)
    
    frame_idx += 1
    print('Loop time: ' + str(time.time()-loop_start))


