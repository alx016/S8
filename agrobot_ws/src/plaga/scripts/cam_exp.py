#!/usr/bin/python3


import rospy
import cv2 as cv
import sys
import signal
import os
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! \
           nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'

def handler(signal_recieved, frame):
    print('EXIT')
    sys.exit(0)

def main():
    rospy.init_node('imageCapture')
    image_publisher = rospy.Publisher('/img', CompressedImage, queue_size=10)

    cap = cv.VideoCapture(cam_port)
    if not cap.isOpened():
        rospy.logerr("Failed to open the camera")
        return
    
    while True:
        _, img = cap.read()

        # Compress the image
        encode_param = [int(cv.IMWRITE_JPEG_QUALITY), 90]
        _, img_buffer = cv.imencode('.jpg', img, encode_param)
        
        # Create a CompressedImage message
        compressed_img_msg = CompressedImage()
        compressed_img_msg.header.stamp = rospy.Time.now()
        compressed_img_msg.format = "jpeg"
        compressed_img_msg.data = np.array(img_buffer).tostring()
        
        # Publish the message
        image_publisher.publish(compressed_img_msg)

        signal.signal(signal.SIGTSTP, handler)  # Detects when CTRL + Z is displayed and finishes the process
        signal.signal(signal.SIGINT, handler)   # Detects when CTRL + C is displayed and finishes the process

if __name__ == '__main__':
    try:
        rospy.loginfo("Camera")
        main()
    except rospy.ROSInterruptException:
        pass

