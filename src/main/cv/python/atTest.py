import numpy as np
import cv2
import os
import math
import pupil_apriltags as apriltag

# basically fixes the intrinsic parameters and is the class that returns the 3D stuff
# printed 3dpose --> tvec (x: left/right, y: up/down, z: front/back), rvec
# max z is 20 feet (detects, but not necessarily accurate); max x is 1 foot on either side
# at 18 in -> max left/right was 4.5 in
class atTest():

    def __init__(self):
        self.camera_matrix = np.load('calibration_data/camera1_matrix.npy')
        self.dist_coeffs = np.load('calibration_data/camera1_dist.npy')
        self.detector = apriltag.Detector(families="tag36h11", nthreads=4) 
        self.world_positions = {
            1: np.array([2.0, 3.0, 0.0]),  # Tag 1 is at (2m, 3m, 0m) in the world frame
            2: np.array([4.0, 0.0, 0.0]),  # Tag 2 is at (4m, 0m, 0m)
        }
        pass

    def calibrate(self, RES, dirpath, square_size, width, height, visualize=False):
        """ Apply camera calibration operation for images in the given directory path. """

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((height*width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

        objp = objp * square_size

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        images = os.listdir(dirpath)
        for fname in images:
            print(fname)
            img = cv2.resize(cv2.imread(os.path.join(dirpath, fname)), RES)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

            if visualize:
                cv2.imshow('img',img)
                cv2.waitKey(0)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.camera_matrix = mtx
        self.dist_coeffs = dist

        np.save('calibration_data/camera1_matrix.npy',mtx)
        np.save('calibration_data/camera1_dist.npy',dist)
        print('Calibration complete')

    def draw_axis_on_image(self, image, camera_matrix, dist_coeffs, rvec, tvec,cvec, size=1):
        try:
            # Define axis length
            length = size

            # 3D axis points in the marker coordinate system
            axis_points_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, -length]])

            # Project 3D points to image plane
            axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, camera_matrix, dist_coeffs)

            # Convert to integer
            axis_points_2d = np.int32(axis_points_2d).reshape(-1, 2)

            # Draw axis lines directly on the image
            cv2.line(image, tuple(axis_points_2d[0]), tuple(axis_points_2d[1]), (0, 0, 255), 2)  # X-axis (red)
            cv2.line(image, tuple(axis_points_2d[0]), tuple(axis_points_2d[2]), (0, 255, 0), 2)  # Y-axis (green)
            cv2.line(image, tuple(axis_points_2d[0]), tuple(axis_points_2d[3]), (255, 0, 0), 2)  # Z-axis (blue)

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.4
            font_thickness = 1
            text_color = (255, 0, 255)  # White color
            text_position = (10, 30)  # Top-left corner coordinates
            # Add text to the image

            text = str(cvec * 39.37) + ' ' + str(tvec)
            cv2.putText(image, text, text_position, font, font_scale, text_color, font_thickness)
            return image
        except Exception as e:
            print(f"An error occurred: {e}")
            return None


def calculate_camera_position(self, detected_tags):
    """
    Calculate the camera position in the world coordinate frame using detected AprilTags.
    """
    camera_positions = []

    for tag_id, (cvec, tvec) in detected_tags.items():
        if tag_id in self.world_positions:
            # World position of the tag
            world_pos = self.world_positions[tag_id]

            # Camera position in the world frame
            camera_pos = world_pos - cvec  # Camera position is inverse of the tag's position
            camera_positions.append(camera_pos)

    if camera_positions:
        # Average camera positions from all detected tags (mean across x, y, z)
        avg_camera_pos = np.mean(camera_positions, axis=0)  # Mean across all positions (x, y, z)
        print(avg_camera_pos)
        return avg_camera_pos
    return None
