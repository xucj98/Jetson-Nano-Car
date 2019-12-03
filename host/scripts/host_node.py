#!/usr/bin/env python3

import rospy
from tracking.msg import TrackingResult
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import cv2
import sys
import numpy as np

def camera_front_callback(data):
    global bridge, img_f, tracking_res

    img = bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
    # print('camera_callback', [(data.header.stamp - tracking_res[i].header.stamp).to_sec() for i in range(len(tracking_res))])
    #for i in range(len(tracking_res)):
    #    if data.header.stamp == tracking_res[i].header.stamp:
    #        cv2.rectangle(img, (tracking_res[i].x1, tracking_res[i].y1), (tracking_res[i].x2, tracking_res[i].y2), (0, 0, 255), 3)
    # cv2.imshow('front', img)
    # cv2.waitKey(1)
    img_f.append({'stamp': data.header.stamp, 'img': img})
    if len(img_f) > 10:
        del img_f[0]

def tracking_result_callback(data):
    global img_f, tracking_res, moiton_pub
    # print('tracking_callback', [(data.header.stamp - img_f[i]['stamp']).to_sec() for i in range(len(img_f))])
    for i in range(len(img_f)):                
        if img_f[i]['stamp'] == data.header.stamp:
            cv2.rectangle(img_f[i]['img'], (data.x1, data.y1), (data.x2, data.y2), (0, 255, 255), 3)
            cv2.imshow('tracking_result', img_f[i]['img'])
            q = cv2.waitKey(1)
            if q == ord('s'):
                motion_msg = Bool()
                motion_msg.data = True
                motion_pub.publish(motion_msg)              
            elif q == ord('p'):
                motion_msg = Bool()
                motion_msg.data = False
                motion_pub.publish(motion_msg)
            break
    tracking_res.append(data)
    if len(tracking_res) > 10:
        del tracking_res[0]
    
if __name__ == '__main__':
    rospy.init_node('host_node', anonymous=True)

    global bridge, img_f, tracking_res, motion_pub
    bridge = CvBridge()
    img_f = []
    tracking_res = []
    rospy.Subscriber('camera/front/compressed', CompressedImage, camera_front_callback)
    rospy.Subscriber('tracking_result', TrackingResult, tracking_result_callback)
    motion_pub = rospy.Publisher('motion', Bool, queue_size=10)

    rospy.spin()

