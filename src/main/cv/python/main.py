from argparse import ArgumentParser
from config import *
from visionInput import VisionInput, find_camera_index
from data_collector import DataCollector
from apriltag import AprilTag
import time
import cv2
import traceback

if AT_USE_CLI_ARGUMENTS:
    parser = ArgumentParser("main.py", description="2473 CV Code")
    id_group = parser.add_mutually_exclusive_group(required=True)
    id_group.add_argument("-i", "--index", help="Manual USB index (when not on raspberry po)")
    id_group.add_argument("-u", "--usb-id", help="USB bus ID (when on raspberry pi)")
    parser.add_argument("cam_name", help="Camera name for calibration files, NetworkTables, etc.")
    args = parser.parse_args()
    index = int(args.index)
    usb_id = args.usb_id
    cam_name = args.cam_name
else:
    index = AT_CAM_INDEX
    usb_id = AT_CAM_USB_ID
    cam_name = AT_CAM_NAME

if ON_RPI:
    index = find_camera_index(usb_id)

    import ntcore
    NETWORK_IDENTITY = "python"
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4(NETWORK_IDENTITY)
    inst.setServerTeam(NETWORKTABLES_TEAM)

    ps_opt = ntcore.PubSubOptions(periodic=0.2)

    table = inst.getTable(f"${cam_name}_table")
    framePub = table.getDoubleTopic("fps_incremented_value").publish(options=ps_opt)
    tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish(options=ps_opt)
    #outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()

input = VisionInput(AT_FOV, AT_INPUT_RES, AT_CAM_HEIGHT, AT_CAM_ANGLE, index)

tag_module = AprilTag(cam_name)
tagData = []

if ON_RPI:
    table = inst.getTable("datatable")

if DATA_COLLECTION_ENABLED:
    # check whether resolution is correct
    initial_frame = input.getFrame()
    data_collector = DataCollector(cam_name, initial_frame.shape)

while True:
    loop_start = time.time()
    try: 
        initial_frame = input.getFrame()
        annotated_frame = initial_frame.copy()
        tagData = tag_module.estimate_3d_pose(initial_frame, annotated_frame, ARUCO_LENGTH_METERS)
        annotated_frame = cv2.resize(annotated_frame, AT_RESIZED_RES)
        if(tagData is None):
            print(f"{cam_name} - tagData none")
        print(f"{cam_name} - tagData.items:", tagData)

        if DATA_COLLECTION_ENABLED:
            data_collector.write_frame(initial_frame, annotated_frame, tagData)
        
        if ON_RPI:
            #when tagData is none a empty frame will be sent over
            framePub.set(initial_frame.sum())
            tagDataPub.set(tagData)
            #outputStreamPub.set(annotated_frame.flatten().tolist())
        else:
            cv2.imshow('result', annotated_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        #time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"{cam_name} - keyboard interrupt")
        framePub.close()
        tagDataPub.close()
        #outputStreamPub.close()
        input.close()
        break
    except Exception as error:
        print(f"{cam_name} - An exception occurred:", error.__class__)
        traceback.print_exc()

    if ON_RPI:
        tagDataPub.set(tagData)
    
    print(f'{cam_name} - Loop time: ' + str(time.time()-loop_start))


