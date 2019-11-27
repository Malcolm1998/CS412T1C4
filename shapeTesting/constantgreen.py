#!/usr/bin/env python
import signal
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import numpy
import time
from sensor_msgs.msg import Image
from enum import Enum

symbol_green_mask_orig = None
symbol_green_mask_good = [['a']]
bridge = cv_bridge.CvBridge()
shutdown_requested = False
h, w, d = 0, 0, 0

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def main():
    global symbol_green_mask_orig, symbol_green_mask_good, h, w, d
    rospy.init_node('green')
    # image_sub = rospy.Subscriber('cv_camera/image_raw',
    #                               Image, image_callback)
    image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, image_callback)
    while not shutdown_requested:
        rospy.spin()
    return

def image_callback(msg):
    global bridge, symbol_green_mask_good, symbol_green_mask_orig, h, w, d
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape

    upper_green = numpy.array([136, 255, 255])
    lower_green = numpy.array([56, 43, 90])
    symbol_green_mask_orig = cv2.inRange(hsv, lower_green, upper_green)
    # blur = cv2.GaussianBlur(symbol_green_mask_orig,(5,5),0)
    blur = cv2.medianBlur(symbol_green_mask_orig, 7)

    symbol_green_mask_good = blur
    cv2.imshow("green window", symbol_green_mask_good)
    cv2.waitKey(3)
    return

if __name__ == '__main__':
    main()
