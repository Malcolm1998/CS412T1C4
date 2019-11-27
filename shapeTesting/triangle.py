#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import numpy
import time
import math
from sensor_msgs.msg import Image
from enum import Enum

symbol_green_mask_orig = None
symbol_green_mask_good = [['a']]
bridge = cv_bridge.CvBridge()
shutdown_requested = False
h, w, d = 0, 0, 0

class Shapes(Enum):
    unknown = -1
    triangle = 3
    square = 4
    pentagon = 5
    circle = 9

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def detect_shape(mask, canvas=None, threshold=100):
    """Detect a shape contained in an image.

    Adappted from: https://stackoverflow.com/questions/11424002/how-to-detect-simple-geometric-shapes-using-opencv
    """
    detected_shapes = []
    moments = []
    thresh = cv2.threshold(mask, 250, 255, cv2.THRESH_BINARY)[1]
    image2, contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    cv2.drawContours(thresh, contours, -1, (255, 255, 0), 100)
    # cv2.imshow("green window", thresh)
    # cv2.waitKey(5)
    contourDict = dict()
    for cnt in contours:
        if cv2.moments(cnt)["m00"] > threshold:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            val = min(len(approx), 9)
            if val in contourDict.keys(): contourDict[val] += 1
            else: contourDict[val] = 1

    return contourDict

def stuff():
    global symbol_green_mask_orig, symbol_green_mask_good, h, w, d
    rospy.init_node('triangle')

    # image_sub = rospy.Subscriber('cv_camera/image_raw',
                                      # Image, image_callback)
    image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, image_callback)


    endDict = {3:0,4:0,5:0,6:0,7:0,8:0,9:0}
    loop = 1000
    count = 0
    while not shutdown_requested and count < loop:
        if symbol_green_mask_good[0][0] == 'a': continue
        symbol_green_mask = symbol_green_mask_good.copy()

        symbol_green_mask[0:h / 4, 0:w] = 0
        symbol_green_mask[3 * h / 4:h, 0:w] = 0
        shapes = detect_shape(symbol_green_mask)
        for key in shapes.keys():
            if key in endDict.keys(): endDict[key] += 1
            else: endDict[key] = 1
        count += 1
    Circle = {3:0,4:0,5:0,6:0,7:66,8:91,9:843}
    Square = {3:0,4:45,5:329,6:466,7:140,8:21,9:0}
    Triangle = {3:29,4:316,5:129,6:411,7:115,8:0,9:0}
    squareScore = 0
    triangleScore = 0
    circleScore = 0
    for key, val in endDict.items():
        squareScore += math.sqrt(pow(Square[key] - val,2))
        triangleScore += math.sqrt(pow(Triangle[key] - val,2))
        circleScore += math.sqrt(pow((Circle[key] - val),2))
    x = (min([squareScore, triangleScore, circleScore]))
    if squareScore == x: print("square")
    elif triangleScore == x: print("triangle")
    elif circleScore == x: print("circle")
    # fileObj = open("notes.txt", 'a')
    # fileObj.write("--------------------------------------------------\n")
    # for key,val in endDict.items():
    #     fileObj.write("{}:{}, ".format(key,val))
    # fileObj.write("\n--------------------------------------------------\n")
    # fileObj.close()
    print(endDict)
    return 'rotate_180'



def image_callback(msg):
    global bridge, symbol_green_mask_good, symbol_green_mask_orig, h, w, d
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape

    upper_green = numpy.array([136, 255, 255])
    lower_green = numpy.array([56, 43, 90])
    symbol_green_mask_orig = cv2.inRange(hsv, lower_green, upper_green)
    # blur = cv2.GaussianBlur(symbol_green_mask_orig,(27,27),0)
    # blur = cv2.blur(symbol_green_mask_orig, (3,3))
    blur = cv2.medianBlur(symbol_green_mask_orig, 7)

    symbol_green_mask_good = blur
    cv2.imshow("orig", symbol_green_mask_good)
    cv2.waitKey(3)
    # cv2.imshow("green window", symbol_green_mask_good)
    # cv2.waitKey(5)
    # upper_green = numpy.array([120, 255, 255])
    # lower_green = numpy.array([50, 100, 100])
    # symbol_green_mask_orig = cv2.inRange(hsv, lower_green, upper_green)
    # blur = cv2.GaussianBlur(symbol_green_mask_orig,(5,5),0)
    # symbol_green_mask_good = blur
    # cv2.imshow("green window", symbol_green_mask_good)
    # cv2.waitKey(3)
    return

if __name__ == '__main__':
    stuff()
