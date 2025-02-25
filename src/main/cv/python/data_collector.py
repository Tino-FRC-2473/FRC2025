import time
import numpy as np
import cv2
import csv
from config import *
from pathlib import Path

class DataCollector:
    def __init__(self, cam_name: str, initial_frame_shape: np.ndarray):
        basePath = Path(__file__).resolve().parent
        dir = f"{basePath}/{DATA_COLLECTION_DIR}"
        Path(dir).mkdir(exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        timestamp = time.ctime(time.time())
        self.mp4_raw = cv2.VideoWriter(f'{dir}/{cam_name} raw {timestamp}.mp4', fourcc, DATA_COLLECTION_FPS, (initial_frame_shape[1], initial_frame_shape[0]))
        self.mp4_annotated = cv2.VideoWriter(f'{dir}/{cam_name} annotated {timestamp}.mp4', fourcc, DATA_COLLECTION_FPS, AT_RESIZED_RES)
        self.csv_file =  open(f"{dir}/{cam_name} tag data {timestamp}.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.frame_id = 0

    def write_frame(self, raw_frame: cv2.typing.MatLike, annotated_frame: cv2.typing.MatLike, tag_data: list):
        self.mp4_raw.write(raw_frame)
        self.mp4_annotated.write(annotated_frame)
        self.csv_writer.writerow([self.frame_id, tag_data])
        self.frame_id += 1
        self.csv_file.flush() # so it's saved instantly

    def __del__(self):
        self.csv_file.close()
        self.mp4_raw.release()
        self.mp4_annotated.release()
