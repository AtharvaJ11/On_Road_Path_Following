#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import *
from collections import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point
import  math
import json


class WP_Publisher:
    def __init__(self):

        self.throttle_scale = 350 / 400 


        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.odometry_sub = rospy.Subscriber("uwb_odom", Odometry, self.odom_cb)
        self.cmd_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)
        self.next_goal = PoseStamped()
        self.current_pose = None
        # self.goals =  [[-4.3, 9.2],
        # [-7.4, 7.8],
        # [-9.3, 7.8],
        # [-13.3, 6,3],   
        # [-17.8, 6.1]
        #         ]

        self.goals = []
        self.k = 0
        self.twist_msg = Twist()
        args = rospy.myargv(argv=sys.argv)
        self.path_dir = args[1] 
        # print(self.path_dir)
        for p in range(10):
            self.twist_msg.linear.x = self.throttle_scale
            self.cmd_pub.publish(self.twist_msg)
        
        
    # def publish_twist(self, event= None):
    #     msg = Twist()
    #     msg = self.twist_msg
    #     self.cmd_pub.publish(msg)
    def read_path(self):
        rospy.loginfo(len(sys.argv))

        # base_dir = rospy.get_param('directory')
        # with open(os.path.join(base_dir, 'xdot.txt'), 'r') as filehandle:

        f = open(self.path_dir + '/case_3_path.json')




        data = json.load(f)
        self.path = Path()
        self.path.header = self.headerFromJSON(data['header'])
        pose_stamped_array = []

        for _ in range(self.headerFromJSON(data['header']).seq):
            pose_stamped = data['poses'][_]
            # print(pose_stamped)
            pose = self.poseStampedFromJSON(pose_stamped['header'],pose_stamped['pose'])
            pose_stamped_array.append(self.poseStampedFromJSON(pose_stamped['header'],pose_stamped['pose']))

            self.goals.append([pose.pose.position.x, pose.pose.position.y])

        self.path.poses = pose_stamped_array


        # print(pose_stamped)

    def odom_cb(self, odometry_msg):

        # msg.header.stamp = rospy.get_time()
        # msg.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)

        self.current_pose = odometry_msg.pose.pose
        self.path_pub.publish(self.path)

        if self.k == len(self.goals):
            for p in range(5):
                self.twist_msg.linear.x = 0
                self.cmd_pub.publish(self.twist_msg)

            rospy.signal_shutdown("DONE")


        if math.sqrt((self.current_pose.position.x - self.goals[self.k][0])**2 + abs(self.current_pose.position.y - self.goals[self.k][1])**2) < 2:
            rospy.loginfo("PREACHED {}".format(self.k))
            self.k+=1


        else:
            self.twist_msg.linear.x = self.throttle_scale#0.4

            self.next_goal.pose.position.x =  self.goals[self.k][0]
            self.next_goal.pose.position.y =  self.goals[self.k][1]
            self.next_goal.pose.position.z =  1.0
            self.goal_pub.publish(self.next_goal)

            # rospy.Timer(rospy.Duration(1.0/1.0), self.publish_twist)
            self.cmd_pub.publish(self.twist_msg)

    def headerFromJSON(self, header_data):
        header = Header()
        header.seq = header_data['seq']
        header.stamp.secs = header_data['stamp']['secs']
        header.stamp.nsecs = header_data['stamp']['nsecs']
        header.frame_id = header_data['frame_id']
        return header
    def poseFromJSON(self, pose_data):
        pose = Pose()

        position = Point()
        position_data = pose_data['position']
        position.x = position_data['x']
        position.y = position_data['y']
        position.z = position_data['z']

        orientation_data = pose_data['orientation']
        orientation = Quaternion()
        orientation.x = orientation_data['x']
        orientation.y = orientation_data['y']
        orientation.z = orientation_data['z']
        orientation.w = orientation_data['w']

        pose.position = position
        pose.orientation = orientation
        return pose
    # pose = poseFromJSON(pose_data)
    #create pose_stamped
    def poseStampedFromJSON(self, header,pose):
        pose_stamped = PoseStamped()
        pose_stamped.pose = self.poseFromJSON(pose)
        pose_stamped.header = self.headerFromJSON(header)
        return pose_stamped



if __name__ == '__main__':
    rospy.init_node('wp_publisher')
    pb = WP_Publisher()
    pb.read_path()
    
    rospy.spin()
