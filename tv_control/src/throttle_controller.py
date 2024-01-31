#!/usr/bin/env python3

from itertools import count
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np
import math
from math import *
from sensor_msgs.msg import Joy



max_right_steer=21000
max_left_steer=19000
initial_brake=2000
full_brake=3000
normal_steer=20000
brake_scale_value=(full_brake-initial_brake)
steer_scale_value=(max_right_steer-normal_steer)

can_move=True
part_brakes=250
part_steer=100
max_throttle=400


joy_msg =  Joy()


throttle_scale_value=max_throttle
max_throttle_scale=0

count_message_mcbSwitchProblemSolution = 0

def twist_clbk(msg):
    global count_message_mcbSwitchProblemSolution 
    global joy_msg,part_steer,steer_scale_value,brake_scale_value,throttle_scale_value,initial_brake,normal_steer,full_brake,can_move,part_brakes,max_throttle_scale,max_throttle
    throttle_value=0
    
    throttle="t0"
    count_message_mcbSwitchProblemSolution = count_message_mcbSwitchProblemSolution + 1
    print(count_message_mcbSwitchProblemSolution)
    count_message_mcbSwitchProblemSolution = count_message_mcbSwitchProblemSolution % 10
    if count_message_mcbSwitchProblemSolution == 1:
        # pub.publish(throttle)
        pass
    brake_value=initial_brake
    throttle_scale_value=max_throttle

    throttle = msg.linear.x
    desired_throttle=int(np.round(throttle*throttle_scale_value))
    desired_brake=abs(int(np.round(throttle*brake_scale_value)))

    throttle_value=desired_throttle
    throttle="t"+str(throttle_value)
    # if joy_msg.buttons[2]:
    if throttle_value >= 0:
        pub.publish(throttle)
    
    if(desired_throttle<0):
        throttle = "t0"
        pub.publish(throttle)
        brake_value=initial_brake+desired_brake
        brake_value=int((math.floor((brake_value-initial_brake)/part_brakes)*part_brakes)+initial_brake)
        if brake_value > 3000:
            brake_value = 3000
        brake="b"+str(brake_value)
        pub.publish(brake) 



    print("throttle: ", throttle, end=" ")
    print("brake: ", brake_value,end="")
    
    print()
    pass
# def joy_callback(msg):
#     global joy_msg
#     joy_msg = msg

if __name__ == '__main__':
    
    
    rospy.init_node('throttle_controller')
    pub= rospy.Publisher('arduino1',String,queue_size=10)
    # steer_pub=rospy.Publisher('steer_arduino',String,queue_size=10)
    # rospy.Subscriber("joy",Joy,joy_callback)
    rospy.Subscriber("/cmd_vel",Twist,twist_clbk)
    rate=rospy.Rate(10)
    rospy.spin()


    