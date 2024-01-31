#!/usr/bin/env python

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
        args_lateral_dict['K_P'] = 0.8
        args_lateral_dict['K_I'] = 0.0
        args_lateral_dict['K_D'] = 0.1

        args_longitudinal_dict = {}
        args_longitudinal_dict['K_P'] = 0.206
        args_longitudinal_dict['K_I'] = 0
        args_longitudinal_dict['K_D'] = 0.515

        self._twist_msg = Twist()
        self._current_pose = None
        self._next_goal = PoseStamped()
        self._target_speed = 0.0
        self._current_speed = 0.0

        #SUBCRIBERS
        # self._odometry_subscriber = rospy.Subscriber("uwb_odom_low_pass", Odometry, self.odometry_cb)
        self._odometry_subscriber = rospy.Subscriber("uwb_odom", Odometry, self.odometry_cb)

        self._waypoint_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.waypoint_cb)

        #PUBLISHERS
        self._steering_command_publisher = rospy.Publisher("/steer", Twist, queue_size=10)

        self._vehicle_controller = VehiclePIDController(
            self, args_lateral=args_lateral_dict, args_longitudinal=args_longitudinal_dict)

    def odometry_cb(self, odometry_msg):
        # with self.data_lock:
        self._current_pose = odometry_msg.pose.pose
        
        # self._current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 + odometry_msg.twist.twist.linear.y ** 2 + odometry_msg.twist.twist.linear.z ** 2) * 3.6
        self.run_step()
        # rospy.loginfo(self._current_speed)
        # print(odometry_msg.twist.twist.linear.x )
    def waypoint_cb(self, goal):
        self._next_goal = goal
        rospy.loginfo("Recieved NEXT GOAL : {}".format(goal))

    def run_step(self):
        if not self._current_pose:
            rospy.loginfo("Waiting for goal")
            return

        target_pose = self._next_goal 
        rospy.loginfo("test")
        
        if self._next_goal.header.frame_id != 'Now stop':
            steering = self._vehicle_controller.run_step(
                self._target_speed, self._current_speed, self._current_pose, target_pose.pose)
        else:
            steering = 0
        self._twist_msg.angular.z = steering
        self._steering_command_publisher.publish(self._twist_msg)
        


class VehiclePIDController(object):

    def __init__(self, node, args_lateral=None, args_longitudinal=None):

        self.node = node
        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = PIDLateralController(**args_lateral)



    def run_step(self, target_speed, current_speed, current_pose, waypoint):

        throttle = self._lon_controller.run_step(target_speed, current_speed)
        steering = self._lat_controller.run_step(current_pose, waypoint)
        # control.steer = -steering
        # control.throttle = throttle
        # control.brake = 0.0
        # control.hand_brake = False
        # control.manual_gear_shift = False

        return steering



class PIDLateralController(object): 

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I

        rospy.loginfo("LATERAL : ")
        rospy.loginfo(self._K_P)
        rospy.loginfo(self._K_I)
        rospy.loginfo(self._K_D)

        self._e_buffer = deque(maxlen=10)
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

    def run_step(self, current_pose, waypoint):
        v_begin = current_pose.position
        quaternion = (

            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        )
        # _, _, yaw = quat2euler(quaternion)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        v_end = Point()
        v_end.x = v_begin.x + math.cos(yaw)
        v_end.y = v_begin.y + math.sin(yaw)

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.position.x -
                        v_begin.x, waypoint.position.y -
                        v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        previous_error = self.error
        self.error = _dot
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -400.0, 400.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        rospy.loginfo("steer: {}".format(np.clip(output, -1.0, 1.0)))
        return np.clip(output, -1.0, 1.0)

class PIDLongitudinalController(object):
    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        rospy.loginfo("LONGITUDINAL : ")
        rospy.loginfo(self._K_P)
        rospy.loginfo(self._K_I)
        rospy.loginfo(self._K_D)

        # self._K_P = 1.0
        # self._K_D = 0.0
        # self._K_I = 0.0

        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

    def run_step(self, target_speed, current_speed):
        previous_error = self.error
        self.error = target_speed - current_speed
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -40.0, 40.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        # print(np.clip(output , 0.0, 1.0))
        return np.clip(output , 0.0, 1.0)


def main(args=None):
    rospy.init_node("controller")
    # local_planner = None
    # update_timer = None
    # try:
    controller = Controller()
    # rospy.on_shutdown(local_planner.emergency_stop)

    # update_timer = local_planner.new_timer(
    #     local_planner.control_time_step, lambda timer_event=None: local_planner.run_step())
    # controller.run_step()_

    
    # update_timer = rospy.Timer(rospy.Duration(local_planner.control_time_step), lambda timer_event=None: local_planner.run_step())
    rospy.spin()

    # except KeyboardInterrupt:
    #     pass

    # finally:
    #     rospy.loginfo('Local planner shutting down.')

if __name__ == "__main__":
    main()