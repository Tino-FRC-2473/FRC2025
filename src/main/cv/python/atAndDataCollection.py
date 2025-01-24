from visionInput import VisionInput
from apriltag import AprilTag
import time
import ntcore
import numpy as np
import cv2
import csv

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)

table = inst.getTable("datatable")
framePub = table.getDoubleTopic("fps_incremented_value").publish()
tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()

FOV = (50.28, 29.16)
RES = (640 , 480)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
tag_module = AprilTag()
ARUCO_LENGTH_METERS = 0.165
pose_list=[]
NUM_TAGS = 22

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('data_collection.mp4', fourcc, 20.0, RES)

fourcc2 = cv2.VideoWriter_fourcc(*'mp4v')
out2 = cv2.VideoWriter('data_collection_annotated.mp4', fourcc2, 20.0, RES)

with open(f"datacollection {time.ctime(time.time())}.csv", mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Frame #", "Pose data"])
    framenum = 0
    while True:
        p = time.time()
        pose_list = [4000 for _ in range(NUM_TAGS * 6)]
        try: 
            frame = input.getFrame()

            annotated_frame = frame.copy()
            tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
            annotated_frame = cv2.resize(annotated_frame, (320,240))
            print("tagData.items:", tagData.items())
            pose_list = [4000 for _ in range(NUM_TAGS * 6)]
            for key, value in tagData.items():
                print("in for loop. key", key, " value: ", value)
                pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
                print("detected pose_list", pose_list)
            

            framePub.set(frame.sum())
            tagDataPub.set(pose_list)
            outputStreamPub.set(annotated_frame.flatten().tolist())

            cv2.imshow('result', annotated_frame)

            out.write(frame)
            out2.write(annotated_frame)
            writer.writerow([framenum, tagData])
            framenum += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            time.sleep(0.02)
        except KeyboardInterrupt:
            print("keyboard interrupt")
            input.close()
            break
        except Exception as error:
            print("An exception occurred:", error)
            print("not pose list:", pose_list)

        table = inst.getTable("datatable")
        tagDataPub.set(pose_list)

    out.release()
    out2.release()