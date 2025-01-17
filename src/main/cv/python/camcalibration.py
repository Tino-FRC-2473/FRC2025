import cv2
import numpy as np
import glob

# Parameters
CHECKERBOARD = (7, 7)
square_size = 0.015  # Size of each square in meters (adjust as needed)

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points in real-world space)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size  # Scale to actual square size

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in the image plane

# Path to the folder where images are stored
CALIB_DIR = 'src/main/cv/'  # Adjust to your directory path

# Load all images from the directory
images = glob.glob(f"{CALIB_DIR}/*.jpg")

# Loop through the images and find corners
for fname in images:
    img = cv2.imread(fname)  # Read the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:  # If corners were found
        objpoints.append(objp)  # Add the object points
        imgpoints.append(corners)  # Add the image points

        # Refine the corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Draw the corners on the image
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        # Show the image with detected corners
        cv2.imshow('Checkerboard', img)
        cv2.waitKey(500)  # Wait for 500 ms to view the result

cv2.destroyAllWindows()  # Close the display window

# Perform the camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the results
print("Camera Matrix:")
print(camera_matrix)
print("Distortion Coefficients:")
print(dist_coeffs)

# Save the calibration data (camera matrix and distortion coefficients)
np.savez('calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)