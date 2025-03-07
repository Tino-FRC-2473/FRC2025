import numpy as np
import cv2
import os
import pupil_apriltags as apriltag
from pathlib import Path
from config import *
from scipy.spatial.transform import Rotation
from collections import OrderedDict
import math



basePath = Path(__file__).resolve().parent

# basically fixes the intrinsic parameters and is the class that returns the 3D stuff
# printed 3dpose --> tvec (x: left/right, y: up/down, z: front/back), rvec
# max z is 20 feet (detects, but not necessarily accurate); max x is 1 foot on either side
# at 18 in -> max left/right was 4.5 in
class AprilTag():
    def __init__(self, cam_name):
        self.camera_matrix = np.load(f'{basePath}/{AT_NPY_DIR}/{cam_name}matrix.npy')
        self.dist_coeffs = np.load(f'{basePath}/{AT_NPY_DIR}/{cam_name}dist.npy')
        self.detector = apriltag.Detector(families="tag36h11", nthreads=4) 
        self.NUM_TAGS = 22
        self.detectedIDs = []

    def estimate_3d_pose(self, image, ARUCO_LENGTH_METERS):
        #getting the last channel of the image (cv2 populates an image w/ 3 channels of the same info)
        gray = image[:, :, 0]

        results = self.detector.detect(gray)

        ids = [r.tag_id for r in results]
        corners = [r.corners for r in results]

        self.detectedIDs = ids

        pose_list = []

        num_tags = len(ids) if ids is not None else 0
        #print(num_tags)
        if num_tags != 0:
            # Estimate the pose of each detected marker
            for i in range(len(ids)):
                # Estimate the pose
                tvec, rvec, cvec= self.estimate_pose_single_marker(corners[i], ARUCO_LENGTH_METERS, self.camera_matrix, self.dist_coeffs)
                
                pose_list.append(ids[i])
                pose_list.extend(cvec)
                
                #offsets for the z and x positions are currently being accounted for by drive
                # original_z = tvec[2]
                # tvec[2] =  original_z + AT_Z_OFFSET

                # original_x = tvec[0]
                # tvec[0] =  (original_x + AT_X_OFFSET)

                pose_list.extend(tvec)
                euler_rvec = self.rotation_vector_to_euler_angles(rvec)
                pose_list.extend(euler_rvec)
                
                # print("euler_rvec: ", euler_rvec)
        
        pose_list = self.sort_tags_distance(pose_list)

        return pose_list
    
    def estimate_pose_single_marker(self, corners, marker_size, camera_matrix, dist_coeffs):
        try:
            # Define the 3D coordinates of the marker corners in the marker coordinate system
            marker_points_3d = np.array([[-marker_size/2, -marker_size/2, 0], [marker_size/2, -marker_size/2, 0], [marker_size/2, marker_size/2, 0], [-marker_size/2, marker_size/2, 0]], dtype=np.float32)
            # Convert image points to float32
            image_points_2d = corners

            # Solve PnP problem to estimate pose
            _, rvec, tvec = cv2.solvePnP(marker_points_3d, image_points_2d, camera_matrix, dist_coeffs)
            R, _ = cv2.Rodrigues(rvec)
            cvec = (-R.T @ tvec).reshape(3)

            rvec = rvec.flatten()
            tvec = tvec.flatten()
            return tvec, rvec, cvec
        
        except Exception as e:
            print(f"An error occurred: {e}")
            return None, None
    
    def rotation_vector_to_euler_angles(self, rvec):
        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        
        # Extract Euler angles (assuming a standard rotation order like XYZ)
        euler_angles = Rotation.from_matrix(R).as_euler("xyz", degrees=False)

        # print("Euler x angle: ", euler_angles[0]) 
        # print("Euler y angle: ", euler_angles[1])
        # print("Euler z angle: ", euler_angles[2])
        
        # #print(self.fix_camera_tilt(euler_angles[0]))
        # print("Pitch in radians: ", rvec[1])
        
        self.fix_camera_tilt(euler_angles[2], rvec[1])
        return euler_angles

    def fix_camera_tilt(self, euler_yaw_angle, pitch_angle):
        robot_yaw = math.atan(math.tan(euler_yaw_angle) * math.cos(pitch_angle))
        #print("Robot yaw value: ", robot_yaw)
        return robot_yaw

    
    # sorts the tags in the list by their hypotenuse (sqrt of x^2 + z^2)
    def sort_tags_distance(self, pose_list):
        # pose list: [id, cvec, cvec, cvec, x, y, z, rvec, rvec, rvec, id, ...]
        # x value = index of id + 4
        # z value = index of id + 6

        hyp_poses = {} # store dictionary in format of {id: hypotenuse}
        for i in range(0, len(pose_list), 10): 
            hyp = math.sqrt(pose_list[i + 4] ** 2 + pose_list[i + 6] ** 2) # hypotenuse = sqrt(x^2 + y^2)
            hyp_poses[pose_list[i]] = hyp
        
        # sort hypotenuse dictionary
        sorted_hyp_dict = OrderedDict(sorted(hyp_poses.items(), key=lambda item: item[1]))

        sorted_pose_list = [] # sort pose list
        for id in sorted_hyp_dict:
            id_index = pose_list.index(id)
            sorted_pose_list.extend(pose_list[id_index:(id_index+10)])
        
        return sorted_pose_list

    def distance_to_station_tag(self, image, marker_size):
        gray = image[:, :, 0]
        results = self.detector.detect(gray)
        # print("results from distance to tag", results)
        corners = [r.corners for r in results]
        # Testing with coral station apriltag
        marker_points_3d = np.array([[-marker_size/2, -marker_size/2, 0], [marker_size/2, -marker_size/2, 0], [marker_size/2, marker_size/2, 0], [-marker_size/2, marker_size/2, 0]], dtype=np.float32)
        #print("length corners", len(corners))
        image_points_2d = corners[0]
        #print(len(image_points_2d))

        _, rvec, tvec = cv2.solvePnP(marker_points_3d, image_points_2d, self.camera_matrix, self.dist_coeffs)

        R, _ = cv2.Rodrigues(rvec)
        # there's a negative for the x position b/c to the left is negative in the opencv2 systems
        list = [-0.130175, 0.903224, 0.0536]
        intake_pos = np.array(list)
        pose_list = R.T @ (tvec - intake_pos)
        # multiplying by negative one b/c of the way that vector adition works
        pose_list[2] = -1 * pose_list[2]
        pose_list[0] = -1 * pose_list[0]

        return pose_list

def calibrate_camera(RES: tuple[int, int], input_dir_relative: Path, output_dir_relative: Path, square_size: int, width: int, height: int, file_name: str, bw_camera: bool, visualize=False):
    """
    gets intrinsic parameters for the camera

    Args: 
    RES (typle): camera frame width, height in pixels
    input_dir_relative (string): path to folder with images being used for calibration (relative to this file)
    output_dir (string): path to folder to save the npy files (relative to this file)
    square_size (int): size of the squares on the checkerboard you are using to calibrate in meters
    width (int): the width of the checkerboard referencing the inner corners of the checkerboard you are using to calibrate (usually 2 less than you actual width)
    height (int): the height of the checerbord referencing the inner corners of the checkerboard you are using to calibrate (usually 2 less than you actual height)
    file_name (string): name of the file that you would like the npy data to be saved to 
    bw_camera (bool): set to true if using a monochrome camera otherwise set to false
    visualize (bool): set to true if you would like to see the calibration images
    """
    bw_camera = True
    # termination criteria
    # cv2.TERM_CRITERIA_EPS is used below in the corner sub pixel function 
    # where the euclidean distance between corners detected in calibration images is compared 
    # and once it reaches the constant epsilon corner sub pixel terminates
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    input_dir = basePath.joinpath(input_dir_relative)
    output_dir = basePath.joinpath(output_dir_relative)

    #Path(output_dir).mkdir(exist_ok=True) # create calibration directory if it doesn't exist
    images = input_dir.iterdir()

    #print(images)

    for i, fname in enumerate(images):
        if not bw_camera:
            img_path = input_dir.joinpath(fname)
            img = cv2.imread(img_path)
            img = cv2.resize(img, RES)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            try:
                img = cv2.imread(input_dir.joinpath(fname), cv2.IMREAD_GRAYSCALE)
                img = cv2.resize(img, RES)
            except Exception as e:
                print(f"Error reading or resizing image {fname}: {e}")
                continue

        # Find the chess board inner corners
        ret, corners = cv2.findChessboardCorners(img, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            print(f"Image #{(i+1):0>2} : {fname} - Found corners")
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        else:
            print(f"Image #{(i+1):0>2} : {fname} - Could not find corners")

        if visualize:
            cv2.imshow('img',img)
            cv2.waitKey(0)

    
    ret, mtx, dist, rvec, tvec = cv2.calibrateCamera(objpoints, imgpoints, img.shape[::-1], None, None)

    np.save(f"{output_dir}/{file_name}matrix", mtx)
    np.save(f"{output_dir}/{file_name}dist", dist)
    print('Calibration complete')
    
