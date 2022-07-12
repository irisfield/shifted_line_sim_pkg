#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from shifted_line_sim_pkg.cfg import LineFollowConfig

# global variables
yaw_rate = Float32()

################### callback ###################

def dynamic_reconfigure_callback(config, level):
    global RC
    RC = config
    return config

def image_callback(camera_image):

    # convert camera_image into an opencv-compatible image
    cv_image = CvBridge().imgmsg_to_cv2(camera_image, "bgr8")

    # resize the image
    cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)

    # convert the image to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # convert the grayscale image to binary (black and white)
    _, bw_image = cv2.threshold(gray_image, RC.white_low, 255, cv2.THRESH_BINARY)

    # find the contours, or the set of coordinates forming a line, in the binary image
    contours, _ = cv2.findContours(bw_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # initialize the variables for computing the centroid's x and y coordinates
    cx = 0
    cy = 0

    # initialize the variable for finding the largest contour by the size of its area
    max_contour = []

    if len(contours) != 0:
        # find the largest contour by its area
        max_contour = max(contours, key = cv2.contourArea)

        # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        M = cv2.moments(max_contour)

        if M["m00"] != 0:
            # compute the centroid's x and y coordinates
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

    if len(max_contour) != 0:
        # draw the obtained contour lines on the original image
        cv2.drawContours(cv_image, max_contour, -1, (0, 0, 255), 10)
    else:
        rospy.loginfo("contour lost")

    # draw a circle at centroid (https://www.geeksforgeeks.org/python-opencv-cv2-circle-method)
    cv2.circle(cv_image, (cx, cy), 8, (180, 0, 0), -1)  # -1 fill the circle

    # offset the x position of the vehicle to follow the lane
    if RC.lane_follow:
        cx += 170

    pub_yaw_rate(cv_image, cx, cy)

    cv2.imshow("Camera View", cv_image)
    cv2.waitKey(3)

################### algorithms ###################

def pub_yaw_rate(image, cx, cy):

    # get the dimension of the image
    height, width = image.shape[0], image.shape[1]

    # compute the coordinates for the center the vehicle's camera view
    camera_center_y = (height / 2)
    camera_center_x = (width / 2)

    # compute the difference between the x and y coordinates of the centroid and the vehicle's camera center
    center_error = cx - camera_center_x

    # In simulation:
    #       less than 3.0 - deviates a little inward when turning
    #                 3.0 - follows the line exactly
    #       more than 3.0 - deviates a little outward when turning
    correction = 3.0 * camera_center_y

    # compute the yaw rate proportion to the difference between centroid and camera center
    angular_z = float(center_error / correction)

    if cx > camera_center_x:
        # angular.z is negative; left turn
        yaw_rate.data = -abs(angular_z)
    elif cx < camera_center_x:
        # angular.z is positive; right turn
        yaw_rate.data = abs(angular_z)
    else:
        # keep going straight
        yaw_rate.data = 0.0

    yaw_rate_pub.publish(yaw_rate)

    return

################### main ###################

if __name__ == "__main__":

    rospy.init_node("line_follow_outer", anonymous=True)

    rospy.Subscriber("/camera_view", Image, image_callback)

    yaw_rate_pub = rospy.Publisher("/yaw_rate", Float32, queue_size=1)

    dynamic_reconfigure_server = Server(LineFollowConfig, dynamic_reconfigure_callback)

    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass
