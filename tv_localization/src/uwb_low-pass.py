#!/usr/bin/env python
# from tkinter import Y
import rospy
from nav_msgs.msg import Odometry
import numpy as np

odom_filter_old = Odometry()
odom_filter_new = Odometry()

odom_old_strength = 0.9
odom_new_strength = 1 - odom_old_strength

flag_begin = 0
log_count =  100
x = np.zeros(log_count,np.float16)
y = np.zeros(log_count,np.float16)
z = np.zeros(log_count,np.float16)

is_initialized = False
def odom_clbk(msg):
    global flag_begin, odom_filter_new, odom_filter_old, odom_new_strength, odom_old_strength, x, is_initialized
    odom_filter_new = msg


    # if flag_begin == 0:
    #     odom_filter_old = odom_filter_new
    #     flag_begin = 1
    if is_initialized == False:
        odom_filter_old = odom_filter_new
        is_initialized = True
        print("Initialized")
    else:
        odom_filter_new.pose.pose.position.x = (odom_new_strength * odom_filter_new.pose.pose.position.x 
                                                    + odom_old_strength * odom_filter_old.pose.pose.position.x)
        odom_filter_new.pose.pose.position.y = (odom_new_strength * odom_filter_new.pose.pose.position.y 
                                                    + odom_old_strength * odom_filter_old.pose.pose.position.y)
        odom_filter_new.pose.pose.position.z = (odom_new_strength * odom_filter_new.pose.pose.position.z 
                                                    + odom_old_strength * odom_filter_old.pose.pose.position.z)

        # odom_filter_new.pose.pose.orientation.x = (odom_new_strength * odom_filter_new.pose.pose.orientation.x 
        #                                             + odom_old_strength * odom_filter_old.pose.pose.orientation.x)
        # odom_filter_new.pose.pose.orientation.y = (odom_new_strength * odom_filter_new.pose.pose.orientation.y 
        #                                             + odom_old_strength * odom_filter_old.pose.pose.orientation.y)
        # odom_filter_new.pose.pose.orientation.z = (odom_new_strength * odom_filter_new.pose.pose.orientation.z 
        #                                             + odom_old_strength * odom_filter_old.pose.pose.orientation.z)
        # odom_filter_new.pose.pose.orientation.z = (odom_new_strength * odom_filter_new.pose.pose.orientation.z 
        #                                             + odom_old_strength * odom_filter_old.pose.pose.orientation.z)

                                                                                                
        odom_filter_old = odom_filter_new

        x[:-1] = x[1:]; x[-1] = msg.pose.pose.position.x
        y[:-1] = y[1:]; y[-1] = msg.pose.pose.position.y
        z[:-1] = z[1:]; z[-1] = msg.pose.pose.position.z
        rospy.loginfo('axis: miu sigma\nx: %s %s\ny: %s %s\nz: %s %s',x.mean(),x.std(),y.mean(),y.std(),z.mean(),z.std())


    pub.publish(odom_filter_new)

if __name__ == "__main__":
    rospy.init_node("uwb_low_pass", anonymous=True)
    rospy.Subscriber("uwb_odom", Odometry, odom_clbk)
    pub = rospy.Publisher("uwb_odom_low_pass", Odometry, queue_size=10)
    # rospy.Rate(10)
    rospy.spin()
