from config import ON_RPI, DRIVER_CAM_USB_ID, DRIVER_CAM_INDEX, DRIVER_CAM_LISTEN_IP, DRIVER_CAM_LISTEN_PORT
from visionInput import find_camera_index
import cv2
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import socket

if ON_RPI:
    index = find_camera_index(DRIVER_CAM_USB_ID)
else:
    index = DRIVER_CAM_INDEX
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
    server_address = (DRIVER_CAM_LISTEN_IP, DRIVER_CAM_LISTEN_PORT) 
    httpd = HTTPServer(server_address, MJPEGStreamHandler)
    print(f'Streaming video at http://{DRIVER_CAM_LISTEN_IP}:{DRIVER_CAM_LISTEN_PORT}/stream.mjpg')
    httpd.serve_forever()

StreamDriverCam()
