import numpy as np
import cv2
import os
import math
import pupil_apriltags as apriltag

# basically fixes the intrinsic parameters and is the class that returns the 3D stuff
# printed 3dpose --> tvec (x: left/right, y: up/down, z: front/back), rvec
# max z is 20 feet (detects, but not necessarily accurate); max x is 1 foot on either side
# at 18 in -> max left/right was 4.5 in
class AprilTag():

    def __init__(self):
        self.camera_matrix = np.load(os.path.dirname(os.path.abspath(__file__)) + '/calibration_data/camera1_matrix.npy')
        self.dist_coeffs = np.load(os.path.dirname(os.path.abspath(__file__)) + '/calibration_data/camera1_dist.npy')
        self.detector = apriltag.Detector(families="tag36h11", nthreads=4) 
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

    def estimate_pose_single_marker(self, corners, marker_size, camera_matrix, dist_coeffs):
        try:
            # Define the 3D coordinates of the marker corners in the marker coordinate system
            marker_points_3d = np.array([[-marker_size/2, -marker_size/2, 0], [marker_size/2, -marker_size/2, 0], [marker_size/2, marker_size/2, 0], [-marker_size/2, marker_size/2, 0]], dtype=np.float32)
            #marker_points_3d = np.array([[0,0,0], [marker_size,0,0], [marker_size, marker_size, 0], [0, marker_size, 0]])
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

    def estimate_3d_pose(self, image, frame_ann, ARUCO_LENGTH_METERS):

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            results = self.detector.detect(gray)
            ids = [r.tag_id for r in results]
            corners = [r.corners for r in results]

            pose_data = {}
            num_tags = len(ids) if ids is not None else 0
            if num_tags != 0:
                # Estimate the pose of each detected marker
                for i in range(len(ids)):
                    # Estimate the pose
                    tvec, rvec, cvec= self.estimate_pose_single_marker(corners[i], ARUCO_LENGTH_METERS, self.camera_matrix, self.dist_coeffs)
                    
                    pose_data[ids[i]] = (cvec, tvec)
                    
                    self.draw_axis_on_image(frame_ann, self.camera_matrix, self.dist_coeffs, rvec, tvec, cvec, 0.1)
            else: 
                return None
            return pose_data


