#!/usr/bin/env python
import numpy as np
import cv2
import sys

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    #  "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    #  "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    #  "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    #  "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

id = 24
type = "DICT_5X5_100"
output_name = type + 'id' + str(id)
print(output_name)

# Verify that the supplied Aruco tag exists and is supported by OpenCV
if ARUCO_DICT.get(type, None) is None:
    print("[INFO] ArUco tag of '{}' is not supported".format(type))
    sys.exit(0)

# Load the Aruco dictionary
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[type])

# Allocate memory for the output Aruco tag and draw the Aruco marker on the output image
print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(type, id))
tag = np.zeros((300, 300, 1), dtype="uint8")
aruco_marker = cv2.aruco.drawMarker(arucoDict, id, 300, tag, 1)

cv2.imwrite("/home/jorge/Documents/clase-ROS/catkin_ws/src/my_robot_pkg/scripts/tags/" + output_name + ".png",
            aruco_marker)
cv2.imshow("ArUco tag", aruco_marker)
cv2.waitKey(0)
