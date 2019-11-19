#!/usr/bin/env python3
# coding: utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from tracking.msg import TrackingResult
from cv_bridge import CvBridge, CvBridgeError

import cv2
 

def callback(data):
    global bridge, res_pub
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("raw" , img)
    cv2.waitKey(1)

    tracking_result = TrackingResult
    tracking_result.x1 = 100
    tracking_result.y1 = 100
    tracking_result.x2 = 540
    tracking_result.y2 = 380
    tracking_result.image_seq = data.header.seq
    pub.publish(tracking_result)
    pass

if __name__ == '__main__':
    rospy.init_node('webcam_display', anonymous=True)
 
    # make a video_object and init the video object
    global bridge, res_pub
    bridge = CvBridge()
    pub = rospy.Publisher('tracking_result', TrackingResult, queue_size=2)
    rospy.Subscriber('camera/front', Image, callback)

    rospy.spin()

