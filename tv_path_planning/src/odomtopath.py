#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import yaml
import json

def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y,indent=4)

class OdomToPath:
    def __init__(self):
        self.rate = rospy.Rate(0.5)
        self.path_pub = rospy.Publisher('path', Path, latch=True, queue_size=10)
        self.odom_sub = rospy.Subscriber('uwb_odom_low_pass', Odometry, self.odom_cb, queue_size=1)
        self.path = Path()
        self.count = 0

    def odom_cb(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.header.seq = self.count
        cur_pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.header.seq = self.count
        self.path.poses.append(cur_pose)
        self.path_pub.publish(self.path)
        self.count += 1
        with open("jun28path3.json", "w") as outfile:
            json_object = msg2json(self.path)
            outfile.write(json_object)
            print("[{},{}],".format(msg.pose.pose.position.x,msg.pose.pose.position.y))
        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odom_to_path')
    odom_to_path = OdomToPath()
    # print(odom_to_path.path)
    rospy.spin()