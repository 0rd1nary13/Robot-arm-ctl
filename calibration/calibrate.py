import cv2
import numpy as np
import glob

# Parameters
chessboard_size = (9, 6)  # Number of internal corners (not squares)


# Prepare object points like (0,0,0), (1,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)


objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load all images
images = glob.glob('handeye_images/*.jpg')  # Adjust your path

if not images:
    print("No images found in 'handeye_images/' directory")
    print("Please ensure you have calibration images with chessboard patterns")
    exit(1)

gray = None  # Initialize gray variable

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Could not read image: {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners)

        # Optional: draw and display the corners
        # cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        # cv2.imshow('Corners', img)
        # cv2.waitKey(100)

# cv2.destroyAllWindows()

if not objpoints:
    print("No chessboard corners found in any images")
    print("Please ensure your images contain a chessboard pattern")
    exit(1)


# Calibrate camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Save results
np.savez("calibration_data.npz", 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs)

print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)
