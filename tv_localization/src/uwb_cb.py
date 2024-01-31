#!/usr/bin/env python3

from multiprocessing.dummy import shutdown
from numpy import empty
import rospy
from nav_msgs.msg import Odometry
# import numpy as np

class combiner:
    def __init__(self):
        self.odom_combined = Odometry()
        self.sub_position = rospy.Subscriber("uwb_odom_positionOnly", Odometry, self.clbk_odom_Position)
        self.sub_orientation = rospy.Subscriber("uwb_odom_orientationOnly", Odometry, self.clbk_odom_Orientation)
        self.odom_pub = rospy.Publisher("uwb_odom", Odometry, queue_size=10)
        self.rate = rospy.Rate(20) # 10hz
        self.can_I_publish = 0
        pass
    def clbk_odom_Position(self,msg):
        # print("pos aayi")
        # print(msg.header)
        self.odom_combined.header =  msg.header
        self.odom_combined.pose.pose.position =  msg.pose.pose.position
        self.can_I_publish = 1
        pass
    def clbk_odom_Orientation(self,msg):
        # print("orn aayi")
        self.odom_combined.header =  msg.header
        self.odom_combined.pose.pose.orientation =  msg.pose.pose.orientation
        pass
    def loop(self):
        while rospy.is_shutdown is not True:
            if self.odom_combined.header.frame_id != "":
                # print(self.odom_combined.header.frame_id)
                # print(69)
                if(self.can_I_publish):
                    rospy.loginfo("Published")
                    self.odom_pub.publish(self.odom_combined)
                    self.can_I_publish = 0
                self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node("uwb_odom", anonymous=True, disable_signals=True)
    print(666)
    # rospy.Subscriber("uwb_odom_positionOnly", Odometry, clbk_odom_Position)
    # rospy.Subscriber("uwb_odom_orientationOnly", Odometry, clbk_odom_Orientation)
    # pub = rospy.Publisher("uwb_odom_low_pass", Odometry, queue_size=10)
    kkk = combiner()
    kkk.loop()
    # rospy.Rate(10)
    rospy.spin()