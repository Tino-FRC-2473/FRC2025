import cv2
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import socket
from linuxpy.video.device import Device

PORT = 1181
LISTEN_IP =  '0.0.0.0' # 0.0.0.0 means accessible from any ip
CAM_USB_ID = "usb-xhci-hcd.0-2" # based on port of raspberry pi (top right)

def find_camera_index(usbId):
    index = 0
    while index < 4: # there can't be more than 4 cameras, I think
        with Device.from_id(index) as cam:
            if cam.info.bus_info == usbId:
                return index

index = find_camera_index(CAM_USB_ID)
camera = cv2.VideoCapture(index)
class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            while True:
                ret, frame = camera.read()
                if not ret:
                    continue

                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue

                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n\r\n')
                time.sleep(0.03)

        else:
            self.send_response(404)
            self.end_headers()


def StreamDriverCam():
    server_address = (LISTEN_IP, PORT) 
    httpd = HTTPServer(server_address, MJPEGStreamHandler)
    print(f'Streaming video at http://{LISTEN_IP}:{PORT}/stream.mjpg')
    httpd.serve_forever()

StreamDriverCam()
