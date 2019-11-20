#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
import sys
import yaml
import os

def camImagePub():
    r = rospkg.RosPack()

    # read config file
    CAM_CFG_PATH = os.path.join(r.get_path('camera'), 'cfg/cam.yaml')
    assert os.path.exists(CAM_CFG_PATH), \
        'Camera configure file does not exist in the following path: {}.'.format(CAM_CFG_PATH)    
    cam_cfg_file = open(CAM_CFG_PATH, 'r')
    cam_cfg = yaml.load(cam_cfg_file.read())
    
    # init ros_node
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(cam_cfg['PUB_RATE'])
 
    if cam_cfg['FRONT_CAM']['USED']:
        front_cap = cv2.VideoCapture(cam_cfg['FRONT_CAM']['NUMBER'])
        front_cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg['FRONT_CAM']['WIDTH'])
        front_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg['FRONT_CAM']['HEIGHT'])
        # front_cap.set(cv2.CAP_PROP_FPS, cam_cfg['PUB_RATE'])
        front_pub = rospy.Publisher('camera/front', Image, queue_size=cam_cfg['QUEUE_SIZE'])
        front_cmprs_pub = rospy.Publisher('camera/front/compress', CompressedImage, queue_size=cam_cfg['QUEUE_SIZE'])
        assert front_cap.isOpened(), 'Front camera is not available!'
        
    if cam_cfg['LEFT_CAM']['USED']:
        left_cap = cv2.VideoCapture(cam_cfg['LEFT_CAM']['NUMBER'])
        left_cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg['LEFT_CAM']['WIDTH'])
        left_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg['LEFT_CAM']['HEIGHT'])
        # left_cap.set(cv2.CAP_PROP_FPS, cam_cfg['PUB_RATE'])
        left_pub = rospy.Publisher('camera/left', Image, queue_size=cam_cfg['QUEUE_SIZE'])
        assert left_cap.isOpened(), 'Left camera is not available!'

    if cam_cfg['RIGHT_CAM']['USED']:
        right_cap = cv2.VideoCapture(cam_cfg['RIGHT_CAM']['NUMBER'])
        right_cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg['RIGHT_CAM']['WIDTH'])
        right_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg['RIGHT_CAM']['HEIGHT'])
        # right_cap.set(cv2.CAP_PROP_FPS, cam_cfg['PUB_RATE'])        
        right_pub = rospy.Publisher('camera/right', Image, queue_size=cam_cfg['QUEUE_SIZE'])
        assert right_cap.isOpened(), 'Right camera is not available!'

    # the 'CVBridge' is a python_class, must have a instance.
    # That means "cv2_to_imgmsg() must be called with CvBridge instance"
    bridge = CvBridge()
 
    while not rospy.is_shutdown():
        if cam_cfg['FRONT_CAM']['USED']:
            _, frame = front_cap.read()
            
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = rospy.get_rostime()
            front_pub.publish(msg)
                
            cmprs_msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            cmprs_msg.header.stamp = rospy.get_rostime()
            front_cmprs_pub.publish(cmprs_msg)
            # cv2.imshow('front', frame)  
    
        if cam_cfg['LEFT_CAM']['USED']:
            _, frame = left_cap.read()
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = rospy.get_rostime()
            left_pub.publish(msg)
            # cv2.imshow('left', frame)
        
        if cam_cfg['RIGHT_CAM']['USED']:
            _, frame = right_cap.read()
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = rospy.get_rostime()            
            right_pub.publish(msg)
            # cv2.imshow('right', frame)
        
        rate.sleep()
 
if __name__ == '__main__':
    try:
        camImagePub()
    except rospy.ROSInterruptException:
        pass

