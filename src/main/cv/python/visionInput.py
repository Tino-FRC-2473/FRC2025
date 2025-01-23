import cv2

def find_camera_index(usbId):
    from linuxpy.video.device import Device
    
    for index in range(4): # there probably won't be more than 4 cameras on the pi
        try:
            with Device.from_id(index) as cam:
                if cam.info.bus_info == usbId:
                    return index
        except:
            print("not connected")
            
    print(f"find_camera_index: Could not find camera on {usbId}")

class VisionInput:
    def __init__(self, fov, res: tuple, cam_height, cam_angle, cam_index):
        self.fov = fov
        self.w = res[0]
        self.h = res[1]
        self.cam_h = cam_height
        self.cam_a = cam_angle
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        

    def getFrame(self):
        if not self.cap.isOpened():
            print("cannot open cam")
        ret, fr = self.cap.read()
        if not ret:
            print('frame malf')
        exit
        print("getting an image")
        return fr


    def close(self):
        self.cap.release()
