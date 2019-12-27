#!/usr/bin/env python3

import rospy
from tracking.msg import TrackingResult
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

import cv2
import sys
import numpy as np

def cam_dprt_l_callback(data):
    global bridge, img_f, tracking_res

    img = bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
    cv2.imshow('disparity', img)
    cv2.waitKey(1)
    print(img.shape)

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)

    global bridge, img_f, tracking_res
    bridge = CvBridge()
    img_f = []
    tracking_res = []
    rospy.Subscriber('camera/left/disparity/compressed', CompressedImage, cam_dprt_l_callback)

    rospy.spin()

