#!/usr/bin/env python3
# coding: utf-8

import rospy
import rospkg
from sensor_msgs.msg import CompressedImage
from tracking.msg import TrackingResult
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import torch

from DaSiamRPN.run_SiamRPN import SiamRPN_init, SiamRPN_track
from DaSiamRPN.net import SiamRPNvot
from DaSiamRPN.utils import cxy_wh_2_rect, rect_2_cxy_wh
from Yolov3.detector import detector
from tracker import HumanTracker


def callback(data):
    global bridge, res_pub, my_tracker, detect_human
    img = bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
  
    if not detect_human:
        d_flag, human = my_tracker.detect(img)
        # cv2.imshow('detecting', img)
        # cv2.waitKey(1)
        if d_flag:
            detect_human = True
            # init_box = [human[0], human[1], human[2]-human[0], human[3]-human[1]]
            # target_pos, target_sz = rect_2_cxy_wh(init_box)
            # state = SiamRPN_init(img, target_pos, target_sz, net)
            my_tracker.init_SOT(human, img)
    else:
        # state = SiamRPN_track(state, img)  # track
        # res = cxy_wh_2_rect(state['target_pos'], state['target_sz'])
        # res = [int(l) for l in res]
        

        res = my_tracker.track(img)
        # cv2.rectangle(img, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)
        # cv2.imshow('SiamRPN', img)
        # cv2.waitKey(1)
        tracking_result = TrackingResult()
        tracking_result.x1 = res[0]
        tracking_result.y1 = res[1]
        tracking_result.x2 = res[0] + res[2]
        tracking_result.y2 = res[1] + res[3]
        tracking_result.header.stamp = data.header.stamp
        res_pub.publish(tracking_result)


if __name__ == '__main__':
    rospy.init_node('tracking_node', anonymous=True)
 
    # make a video_object and init the video object
    global bridge, res_pub, yolo, detect_human, net
    # init yolov3, video
    my_tracker = HumanTracker()
    detect_human = False

    r = rospkg.RosPack()
    # net = SiamRPNvot()
    # net.load_state_dict(torch.load(r.get_path('tracking') + '/scripts/DaSiamRPN/SiamRPNVOT.model'))
    # net.eval().cuda()
    torch.cuda.synchronize()

    bridge = CvBridge()
    res_pub = rospy.Publisher('tracking_result', TrackingResult, queue_size=5)
    rospy.Subscriber('camera/front/compressed', CompressedImage, callback, queue_size=1, buff_size=1000000)

    rospy.spin()

