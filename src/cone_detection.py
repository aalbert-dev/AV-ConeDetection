#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from std_msgs.msg import String
import numpy as np
import cv2 as cv

# constants
PUBLISH_ROS_IMGS = True
BRIDGE = CvBridge()
CONTOUR_SIZE_THRESHOLD = 0

# callback for each camera frame
def rgb_cb(msg):

    # convert image to opencv readable and detect cones
    cv_image = BRIDGE.imgmsg_to_cv2(msg)
    detect_cones(cv_image)

def detect_cones(img):

    #       APPROACHES
    # hough lines detection -- too many lines and not alligned with cones
    # canny edge detection -- too many lines and too slow
    # YOLO other machine learning -- could not figure out how to train and don't have enough data rn
    # classic contour detection -- did not produce diagnol lines for some reason (maybe human error)
    # canny edge detection again (point method) -- produced points but hard to obtain lines for cone from points
    # finally use hsv image to produce mask to use on cone color!!!

    # HSV = Hue, saturation, and value color model

    # get hsv image from opencv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) 

    # TODO  keep tuning blue color bounds
    lower_color_blue_bound = np.array([110,125,125]) 
    upper_color_blue_bound = np.array([130,255,255]) 
  
    # find pixels in range bounded by BGR color bounds
    mask = cv.inRange(hsv, lower_color_blue_bound, upper_color_blue_bound)

    # find pixels that are in both mask AND original img
    masked_img = cv.bitwise_and(img, img, mask=mask)
    
    # create blob detection params
    params = cv.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 256
    params.filterByArea = True
    params.minArea = 250
    params.maxArea = 20000
    # params.filterByColor=True
    # params.blobColor=235
    params.filterByInertia = False
    params.filterByConvexity = False
    params.filterByColor = False
    params.filterByCircularity = False

    # add keypoints for blob bounding box to image
    detector = cv.SimpleBlobDetector_create(params)
    keypoints = detector.detect(masked_img)

    # draw each blob as a circle around center point with radius scaling by size of blob 
    im_with_keypoints = cv.drawKeypoints(masked_img, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # draw depths for each blob
    if last_depth_image is not None:
        for kp in keypoints:
            pt = kp.pt; x = int(pt[0]); y = int(pt[1]); 
            w, h, l = img.shape
            if x < w - 1 and y < h - 1 and x > 0 and y > 0:

                # sample 9 points to find average distance to object
                z = last_depth_image[x][y]
                a = last_depth_image[x][y + 1]
                b = last_depth_image[x][y - 1]
                c = last_depth_image[x + 1][y]
                d = last_depth_image[x + 1][y + 1]
                e = last_depth_image[x + 1][y - 1]
                f = last_depth_image[x - 1][y]
                g = last_depth_image[x - 1][y + 1]
                h = last_depth_image[x - 1][y - 1]
                new_z = z + a + b + c + d + e + f + g + h
                new_z /= 9
            else:
                z = "nan"
            
            # add range as text to image
            cv.putText(im_with_keypoints, str(z), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))



    # republish image hsv and masked image
    ros_img = BRIDGE.cv2_to_imgmsg(im_with_keypoints)
    rgb_pub.publish(ros_img)    
    ros_img = BRIDGE.cv2_to_imgmsg(hsv)
    hsv_pub.publish(ros_img)

# callback for each frame of depth image
def depth_cb(msg):
    cv_image = BRIDGE.imgmsg_to_cv2(msg)
    global last_depth_image
    last_depth_image = cv_image

# data structures
last_depth_image = None


# declare as node
rospy.init_node('cone_detection')

# subscriber/publishers
depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depth_cb)
rgb_sub = rospy.Subscriber('/usb_cam/image_raw', Image, rgb_cb)
rgb_pub = rospy.Publisher('/camera/rgb/image_detection', Image, queue_size=1)
hsv_pub = rospy.Publisher('/camera/rgb/hsv_image', Image, queue_size=1)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
cone_locations_pub = rospy.Publisher('cone_locations', String, queue_size=10)

# control loop
while not rospy.is_shutdown():
    rospy.sleep(10)
