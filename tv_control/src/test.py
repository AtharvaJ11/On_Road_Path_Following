#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

def joy_cb(self,joy_msg):
    if(joy_msg.buttons[2]==1):
        self.canMove=True
    else:
        self.canMove=False
    rospy.loginfo("Kill movement: {}".format(self.canMove))

def steer_cb(self,steer_msg):
    if self.canMove is True:
        self.steer_pub.publish(steer_msg)
    else:
        #publish zero steer
        pass

def throttle_cb(self,throttle_msg):
    if self.canMove is True:
        self.th_pub.publish(throttle_msg)
        
    else: 
        #publish zero throttle