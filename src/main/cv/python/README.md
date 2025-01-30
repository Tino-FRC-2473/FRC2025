# CV Code
## Setup
Create the virtual environment:
```
python -m venv venv
```
Activate it **for macOS/Linux**:
```
source venv/bin/activate
```
Activate it **for Windows**:
```
.\venv\Scripts\activate
```
Install the required files **for Raspberry Pi/ARM coprocessor only**
```
pip install -r requirements_rpi.txt
```
Install the required files **for other operating systems**
```
pip install -r requirements.txt
```
Copy the config file from `config.default.py` to `config.py` and change it if needed (note `ON_RPI`).

## USB IDs
The USB XHCI ID is based on the port it's plugged into. When facing the Pi on a Pi 5:

| Position     | Id                 |
| ------------ | ------------------ |
| Top-Left     | `usb-xhci-hcd.1-1` |
| Top-Right    | `usb-xhci-hcd.0-2` |
| Bottom-Left  | `usb-xhci-hcd.0-1` |
| Bottom-Right | `usb-xhci-hcd.1-2` |
 
It will be different for other Pi's. To find out the id for a specific port, run
```
v4l2-ctl --list-devices
```
and you will see something like the following:
```
Arducam OV9281 USB Camera: Ardu (usb-0000:01:00.0-1.4):
	/dev/video0
	/dev/video1
	/dev/media4
```
In this case `usb-0000:01:00.0-1.4` is your id.

## Running AprilTag Code
```
Make sure that self.camera_matrix and self.dist_coffes reference the calibration data files
```
Change CALIB_DIR in config.py to be the folder that contains the version of the code locally on yoru computer
```
Change AT_CAM_NAME to the name of the camera you are using and the version of the calibration data you are saving if that is applicable
```


## Running Calibration Code for the Apriltag
```
Make sure to go to apriltag.py and comment our the self.dist_coeffs and self.camera_matrix as you can't load calibration files if you are still trying to collect calibration data and save it to certain files
```
