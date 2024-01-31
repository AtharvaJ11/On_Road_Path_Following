#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Path

pos_array = [[0,0],
                [0,1],
                [0,2],
                [0.5,2],
                [1,2],
                [1.5,2]]

def array2path(pos_array):
    path = Path()
    path.header.frame_id = 'path header #777'
    pose_stamped_array = []
    i = 0
    for pos in pos_array:
        i+=1
        pose_stamped = PoseStamped() 
        pose_stamped.header.seq = i
        pose_stamped.header.frame_id = 'poses header #666'
        pose_stamped.pose.position.x = pos[0]
        pose_stamped.pose.position.y = pos[1]
        pose_stamped_array.append(pose_stamped)
    path.poses = pose_stamped_array
    print(path)
    return path

array2path(pos_array)
