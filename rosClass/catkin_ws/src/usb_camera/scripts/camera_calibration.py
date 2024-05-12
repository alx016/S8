
import numpy as np
import cv2
import glob
import sys
import os
import ruamel.yaml

chessboardSize = (9, 6)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0]*chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

folder = "snapshots/"

try:
    os.mkdir(folder)
except OSError as error:
    print(error)

images = glob.glob(folder + "*.png")

i = 1
for fname in images:
    msg = "Reading image: {}. Progress: {:.2f}%".format(fname[10:-4], 100*i/len(images))
    i = i+1

    print(msg, end ='\r')

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (chessboardSize[0], chessboardSize[1]), None)

    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)

        #Draw and display the corners
        cv2.drawChessboardCorners(img, (chessboardSize[0], chessboardSize[1]), corners2, ret)
        cv2.imwrite(folder + "pattern_{}.jpg".format(fname[10:-4]), img)
        cv2.imshow('img', img)
        cv2.waitKey(500)

ret, cam_mtx, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Camera Matrix: ")
print(cam_mtx)
print("Distorsion coefficients: ")
print(distCoeffs)


img = cv2.imread(folder + 'img_1.png')
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, distCoeffs, (w, h), 1, (w, h))

#Option 2
mapx, mapy = cv2.initUndistortRectifyMap(cam_mtx, distCoeffs, None, newcameramtx, (w, h), 5)
dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imshow("Calibration Result - opt 2", dst)
cv2.imwrite("Calibration_result_opt2.png", dst)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cam_mtx, distCoeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print("Total error: {}".format(mean_error/len(objpoints)))

yaml_str = """
#Camera calibration paramters
camera_intrinsic_matrix:
    fx: fx value #Focal length in the x-direction (pixels)
    fy: fy value #Focal length in the y-direction (pixels)
    cx: cx value #Optical center in x-axis (image_center)
    cy: cy value #Optical center in y-axis (image_center)
distorsion_coefficients:
    k1: k1_value #First-order radial distorsion coefficient
    k2: k2_value #Second-order radial distortion coefficient
    p1: p1_value #First-order tangential distortion coefficient
    p2: p2_value #Second-order tangential distortion coefficient
    k3: k3_value #Third-order radial distortion coefficient
"""

yaml = ruamel.yaml.YAML()
data = yaml.load(yaml_str)

#Store the calibration parameters
data['camera_intrinsic_matrix']['fx'] = float(cam_mtx[0, 0])
data['camera_intrinsic_matrix']['fy'] = float(cam_mtx[1, 1])
data['camera_intrinsic_matrix']['cx'] = float(cam_mtx[0, 2])
data['camera_intrinsic_matrix']['cy'] = float(cam_mtx[1, 2])
data['distorsion_coefficients']['k1'] = float(distCoeffs[0][0])
data['distorsion_coefficients']['k2'] = float(distCoeffs[0][1])
data['distorsion_coefficients']['p1'] = float(distCoeffs[0][2])
data['distorsion_coefficients']['p2'] = float(distCoeffs[0][3])
data['distorsion_coefficients']['k3'] = float(distCoeffs[0][4])

#yaml.dump(data, sys.stdout)

filename = "../config/calibration_params.yaml"

with open(filename, "w") as yaml_file:
    yaml.dump(data, yaml_file)

print("Calibration parameters saved to {}".format(filename))

cv2.waitKey()
cv2.destroyAllWindows()
