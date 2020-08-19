#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv

# constants
PUBLISH_ROS_IMGS = True
BRIDGE = CvBridge()                 
CONTOUR_SIZE_THRESHOLD = 0          # pixels ^ 2

# callback for each camera frame
def cv_cb(msg):
    cv_image = BRIDGE.compressed_imgmsg_to_cv2(msg)
    actual_detect_line(cv_image)

def actual_detect_line(img):

    # flip image to adjust for camera
    img = cv.flip(img, -1)

    # find lines parameters
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    kernel_size = 5
    blur_gray = cv.GaussianBlur(gray,(kernel_size, kernel_size),0)
    low_threshold = 75
    high_threshold = 200
    edges = cv.Canny(blur_gray, low_threshold, high_threshold)
    rho = 1 
    theta = np.pi / 180  
    threshold = 50 
    min_line_length = 20
    max_line_gap = 30
    line_image = np.copy(img) * 0 
    lines = cv.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
    line_centers = []

    # if there are lines
    if lines is not None and len(lines) > 0:
        for line in lines:
            for x1,y1,x2,y2 in line:

                # get coordinates of line
                m = (y2 - y1) / (x2 - x1)
                a = round(np.rad2deg((math.atan(m))), 2)
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2

                font = cv.FONT_HERSHEY_SIMPLEX 
                
                # fontScale 
                fontScale = 0.3
                
                # Blue color in BGR 
                color = (255, 0, 0) 
                
                # Line thickness of 2 px 
                thickness = 2
                cv.putText(img, str(a), (mid_x, mid_y), font, fontScale, color, thickness, cv.LINE_AA) 
                line_centers.append((mid_x, mid_y))
                cv.line(img,(x1, y1),(x2,y2),(255,0,0),5)
                cv.circle(img, (mid_x, mid_y), 5, (0, 255, 0))

    second_line_centers = []

    # republish image to ros for visualations
    ros_img = BRIDGE.cv2_to_imgmsg(img, 'rgb8')
    img_pub.publish(ros_img)

# calculate distance between two points
def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return ((x2 - x1)**2 + (y2 - y1)**2)**(0.5)

# draw bounding boxes
def detect_line(img):

    # correct image rotation
    img = cv.flip(img, -1)

    # gray scale + blur image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray,(5,5),0)
    ret, thresh = cv.threshold(blur,60,255,cv.THRESH_BINARY_INV)

    # get contours from image
    a, contours, b = cv.findContours(thresh.copy(), 1, cv.CHAIN_APPROX_NONE)

    # if any contours exist
    if len(contours) > 0:

        # draw bounding box for each contour
        for c in contours:

            # get coordinates of bb
            x, y, w, h = cv.boundingRect(c)

            # if contour is significant
            if w * h > CONTOUR_SIZE_THRESHOLD:
                

                # publish image with line detection to ROS
                if PUBLISH_ROS_IMGS:
                    cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0))
                    ros_img = BRIDGE.cv2_to_imgmsg(img, 'rgb8')
                    img_pub.publish(ros_img)


# subscriber/publisher declarations
cam_sub = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, cv_cb)
img_pub = rospy.Publisher('cv_bb_image', Image, queue_size=1)

rospy.init_node('contour_display')

# update at 10 hz
rate = rospy.Rate(10)

# control loop
rospy.spin()
