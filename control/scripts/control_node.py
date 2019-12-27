#!/usr/bin/env python3

import rospy
from tracking.msg import TrackingResult
from std_msgs.msg import Bool
from std_msgs.msg import Int16

from controller import Controller, Mode

def tracking_result_callback(data):
    global init_size, controller, motion, left_depth, right_depth
    
    if init_size is None:
        init_size = (data.x2 - data.x1) * (data.y2 - data.y1)
    
    ds = 1 - (data.x2 - data.x1) * (data.y2 - data.y1) / init_size
    dy = ((data.y1 + data.y2) / 2 - 240) / 240
    dx = ((data.x1 + data.x2) / 2 - 320) / 320

    # ds = 0 if(ds < 0.2) else ds
    speed = (ds + dy + abs(dx)) * 70
    # speed = ds * 70

    if speed < controller.MIN_SPEED:        
        speed = 0
    else:
        speed = max(controller.MIN_SPEED, min(speed, 45))
    speed = int(speed)
    
    angle = ((data.x1 + data.x2) / 2 - 320) / 320
    # angle = ((data.x1 + data.x2) / 2 - 320) / 200
    # angle = angle + max((1000 - left_depth) / 400, 0)
    # angle = angle - max((1000 - right_depth) / 400, 0)
    # angle = angle if(abs(angle) > 0.1) else 0
    if angle > 0.2:
        speed = int(speed * 0.8)
    sign = 1 if(angle >= 0) else -1
    angle = max(-1, min(1, angle))
    angle = 90 + (abs(angle) ** 1.2) * 25 * sign
    print(angle)
    angle = max(controller.MIN_SERVO_ANGLE, min(controller.MAX_SERVO_ANGLE, angle))
    angle = int(angle)

    if motion and (left_depth > 900 or right_depth > 900):
        mode = Mode.FORWARD
    else:
        mode = Mode.STOP
    controller.set_motion(speed, angle, mode)

def motion_callback(data):
    global motion
    motion = data.data

def left_depth_callback(data):
    global left_depth
    left_depth = data.data
    if left_depth == 10000:
        left_depth = 622

def right_depth_callback(data):
    global right_depth
    right_depth = data.data
    if right_depth == 10000:
        right_depth = 622

if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    
    global init_size, controller, motion, left_depth, right_depth
    init_size = None
    controller = Controller()
    motion = False
    left_depth = 100000
    right_depth = 100000
    
    rospy.Subscriber('tracking_result', TrackingResult, tracking_result_callback)
    rospy.Subscriber('motion', Bool, motion_callback)
    rospy.Subscriber('camera/left/depth', Int16, left_depth_callback)
    rospy.Subscriber('camera/right/depth', Int16, right_depth_callback)
    rospy.spin()
