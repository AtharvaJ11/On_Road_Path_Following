#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
# import math
from math import *

# joy_msg=Joy()
max_right_steer=21000
max_left_steer=19000
normal_steer=20000
steer_scale_value=(max_right_steer-normal_steer)

part_steer=100

def controller_clbk(msg):
    global part_steer,steer_scale_value,normal_steer
    steer_cmd_value = msg.angular.z#msg.data
    print(steer_cmd_value)
    if steer_cmd_value > 1:
        steer_cmd_value = 1
    elif steer_cmd_value < -1:
        steer_cmd_value =-1
    desired_steer=-1*int(np.round((steer_cmd_value**1)*steer_scale_value))*np.sign(steer_cmd_value)
    steer_value=normal_steer+desired_steer
    steer_value=steer_value-(steer_value%part_steer)
    steer="s"+str(steer_value)
    steer_pub.publish(steer)

if __name__ == '__main__':
    
    
    rospy.init_node('drive_joystick666')
    # pub= rospy.Publisher('arduino',String,queue_size=10)
    steer_pub=rospy.Publisher('steer_arduino1',String,queue_size=10)
    # rospy.Subscriber("joy",Joy,joy_callback)
    rospy.Subscriber("/steer",Twist,controller_clbk)
    rate=rospy.Rate(10)
    rospy.spin()