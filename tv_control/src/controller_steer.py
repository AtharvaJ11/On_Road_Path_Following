#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String, Bool, Float32
from collections import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point
import  math


class Controller:
    def __init__(self):
        args_lateral_dict = {}
        # args_lateral_dict['K_P'] = 0.8
        # args_lateral_dict['K_I'] = 0.0
        # args_lateral_dict['K_D'] = 0.1
        
        self._K_P = 1.5
        self._K_D = 0.
        self._K_I = 0.

        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

        # args_longitudinal_dict = {}
        # args_longitudinal_dict['K_P'] = 0.206
        # args_longitudinal_dict['K_I'] = 0
        # args_longitudinal_dict['K_D'] = 0.515

        self._twist_msg = Twist()
        self._current_pose = None
        self._next_goal = PoseStamped()
        self._target_speed = 0.0
        self._current_speed = 0.0

        self.first_target_recieved = False
        self.rate = rospy.Rate(20)

        #SUBCRIBERS
        # self._odometry_subscriber = rospy.Subscriber("uwb_odom_low_pass", Odometry, self.odometry_cb)
        self._odometry_subscriber = rospy.Subscriber("uwb_odom", Odometry, self.odometry_cb)

        self._waypoint_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.waypoint_cb)

        #PUBLISHERS
        self._steering_command_publisher = rospy.Publisher("/steer", Twist, queue_size=10)

        # self._vehicle_controller = VehiclePIDController(
        #     self, args_lateral=args_lateral_dict, args_longitudinal=args_longitudinal_dict)

    def odometry_cb(self, odometry_msg):
        # with self.data_lock:
        self._current_pose = odometry_msg.pose.pose
    def waypoint_cb(self, goal):
        self._next_goal = goal
        if self.first_target_recieved != True:
            rospy.loginfo("Recieved NEXT GOAL : {}".format(goal))
            self.first_target_recieved = True

    def loop(self):
        while rospy.is_shutdown is not True:
            if not self._current_pose:
                rospy.loginfo("Waiting for pose")
            elif not self._next_goal:
                rospy.loginfo("Waiting for goal")
            else:

                rospy.loginfo("Namaste")
                target_pose = self._next_goal 
                ###############
                v_begin = self._current_pose.position
                quaternion = (

                    self._current_pose.orientation.x,
                    self._current_pose.orientation.y,
                    self._current_pose.orientation.z,
                    self._current_pose.orientation.w
                )
                # _, _, yaw = quat2euler(quaternion)
                (roll, pitch, yaw) = euler_from_quaternion(quaternion)

                v_end = Point()
                v_end.x = v_begin.x + math.cos(yaw)
                v_end.y = v_begin.y + math.sin(yaw)

                v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
                w_vec = np.array([target_pose.pose.position.x -
                                v_begin.x, target_pose.pose.position.y -
                                v_begin.y, 0.0])
                _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                        (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

                _cross = np.cross(v_vec, w_vec)
                if _cross[2] < 0:
                    _dot *= -1.0

                previous_error = self.error
                self.error = _dot
                # restrict integral term to avoid integral windup
                self.error_integral = np.clip(self.error_integral + self.error, -4.0, 4.0)
                self.error_derivative = self.error - previous_error
                output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
                rospy.loginfo("steer: {}".format(np.clip(output, -1.0, 1.0)))
                steering = np.clip(output, -1.0, 1.0)
                ###############


                self._twist_msg.angular.z = steering
                self._steering_command_publisher.publish(self._twist_msg)
            self.rate.sleep()
        
        pass

def main():
    rospy.init_node("controller666", disable_signals=True)
    controller = Controller()
    controller.loop()
    rospy.spin()
    pass

if __name__ == "__main__":
    main()