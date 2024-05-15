#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import sys
import numpy as np
import yaml

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

global aruco_type, marker_size, arucoDict, arucoParams
aruco_type = "DICT_5X5_100"
marker_size = 600   #In Pixels

global cam_mtx, distCoeffs
font_color = (0, 0, 0)
box_color = (255, 0, 255)

def my_estimate_pose_single_marker(corners, mtx, distortion):
    rvecs = []
    tvecs = []

    marker_points = np.array([
    [-marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)
    
    for corner in corners:
        # Ignore return values not used in the original function
        _, R, t = cv2.solvePnP(marker_points, corner, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs

def main_process(image):

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    equalized_image = cv2.equalizeHist(gray_image)

    clahe = cv2.createCLAHE(clipLimit=0.5, tileGridSize=(8, 8))
    clahe_image = clahe.apply(gray_image)
    # image = clahe_image

    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    # detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    # corners, ids, rejectedImgPoints = detector.detectMarkers(clahe_image)

    #Verify *at least* one ArUco marker was detected
    if len(corners) > 0:

        # Flatten the ArUco IDs list
        ids = ids.flatten()

        #Loop over the detected ArUco corners
        for (markerCorner, markerID) in zip(corners, ids):
            # Extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
            corners= markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, box_color, 2)
            cv2.line(image, topRight, bottomRight, box_color, 2)
            cv2.line(image, bottomRight, bottomLeft, box_color, 2)
            cv2.line(image, bottomLeft, topLeft, box_color, 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] -15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color, 2)
            # print("[INFO] ArUco marker ID: {}".format(markerID))

            # rvec, tvec = my_estimate_pose_single_marker(markerCorner, cam_mtx, 0.02)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, cam_mtx, distCoeffs)

            # Ensure rvec and tvec are NumPy arrays
            if not isinstance(rvec, np.ndarray):
                rvec = np.array(rvec)
            if not isinstance(tvec, np.ndarray):
                tvec = np.array(tvec)
            cv2.drawFrameAxes(image, cam_mtx, distCoeffs, rvec, tvec, 0.02)

            rvec_degrees = np.degrees(rvec)
            rvec_degrees = np.round(rvec_degrees).astype(int)


            text = "R: {}, P: {}, Y: {}".format(rvec_degrees[0][0][0], rvec_degrees[0][0][1], rvec_degrees[0][0][2])
            cv2.putText(image, str(text), (bottomRight[0], bottomRight[1] +15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color, 2)
            

    cv2.imshow("ArUco detection", image)
    cv2.waitKey(1)


def imageCallback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Display the image
        # cv2.imshow("Image Subscriber PY", cv_image)
        # cv2.waitKey(1)
        main_process(cv_image)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

if __name__=="__main__":

    #ROS Configuration
    rospy.init_node("detecting_ArUcos_node_py")
    rospy.Subscriber("/camera/image_raw", Image, imageCallback)

    # ArUco configuration
    # Verify that the supplied ArUco tag exists and is supported by OpenCV
    if ARUCO_DICT.get(aruco_type, None) is None:
        rospy.loginfo("ArUco tag of '{}' is not supported".format(aruco_type))
        sys.exit(0)

    # Load the ArUco dictionary and grab the ArUco parameters
    rospy.loginfo("Detecting '{}' tags ... ".format(aruco_type))
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    arucoParams = cv2.aruco.DetectorParameters_create()

    # filename = "src/usb_camera/config/calibration_params.yaml"

    # with open(filename, 'r') as file:
    #     data = yaml.safe_load(file)

    # fx = data['camera_intrinsic_matrix']['fx']
    # fy = data['camera_intrinsic_matrix']['fy']
    # cx = data['camera_intrinsic_matrix']['cx']
    # cy = data['camera_intrinsic_matrix']['cy']

    # k1 = data['distorsion_coefficients']['k1']
    # k2 = data['distorsion_coefficients']['k2']
    # p1 = data['distorsion_coefficients']['p1']
    # p2 = data['distorsion_coefficients']['p2']
    # k3 = data['distorsion_coefficients']['k3']

    #Read camera calibration parameters from ROS parameter server
    fx = rospy.get_param("/camera_intrinsic_matrix/fx")
    fy = rospy.get_param("/camera_intrinsic_matrix/fy")
    cx = rospy.get_param("/camera_intrinsic_matrix/cx")
    cy = rospy.get_param("/camera_intrinsic_matrix/cy")
    k1 = rospy.get_param("/distorsion_coefficients/k1")
    k2 = rospy.get_param("/distorsion_coefficients/k2")
    p1 = rospy.get_param("/distorsion_coefficients/p1")
    p2 = rospy.get_param("/distorsion_coefficients/p2")
    k3 = rospy.get_param("/distorsion_coefficients/k3")

    print("Camera Intrinsic Matrix: ")
    print("fx: {}".format(fx))
    print("fy: {}".format(fy))
    print("cx: {}".format(cx))
    print("cy: {}".format(cy))

    print("\nDistortion Coefficients:")
    print("k1: {}".format(k1))
    print("k2: {}".format(k2))
    print("p1: {}".format(p1))
    print("p2: {}".format(p2))
    print("k3: {}".format(k3))

    cam_mtx = np.array([
        [fx, 0, cx], 
        [0, fy, cy], 
        [0, 0, 1]
    ])

    distCoeffs = np.array([
        [k1, k2, p1, p2, k3]
    ])

    print("Camera Matrix:")
    print(cam_mtx)

    print("\nDistortion Coefficients:")
    print(distCoeffs)

    rospy.spin()