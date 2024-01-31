#!/usr/bin/env python3
import rospy
import json
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from rosgraph_msgs.msg import Clock
import pandas as pd

odom = Odometry()
float64 = Float64()
twist = Twist()
clock = Clock()
timeClock = 0.
yaw_ = 0
goal = [0,0]

def odom_clbk(msg):
    global odom, yaw_
    odom = msg
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def float64_clbk(msg):
    global float64
    float64 = msg

def twist_clbk(msg):
    global twist
    twist = msg

def posestamped_clbk(msg):
    global goal
    goal = [msg.pose.position.x, msg.pose.position.y]

def clock_clbk(msg):
    global clock, timeClock
    clock = msg
    timeClock = clock.clock.secs + clock.clock.nsecs/ 1000000000
    # print("time: {}".format(time))

if __name__ == "__main__":
    rospy.init_node("I_am_an_observer", disable_signals=True)
    rospy.Subscriber("/uwb_odom_low_pass", Odometry, odom_clbk)
    rospy.Subscriber("/steering_command", Float64, float64_clbk)
    rospy.Subscriber("/cmd_vel", Twist, twist_clbk)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, posestamped_clbk)
    rospy.Subscriber("clock", Clock, clock_clbk)

    filename = input()



    time = [0.]
    pos_x = [0.]
    pos_y = [0.]
    goalx = [0.]
    goaly = [0.]
    yaw = [0.]
    steer = [0.]
    throttle = [0.]
    time_is_initialized = False
    while True:
        if odom.pose.pose.position.x != 0 and goal[0] != 0:
            if time_is_initialized is False:
                time_zero = timeClock
                time.append(0)
                time_is_initialized = True
            else:
                time.append(timeClock - time_zero)
            pos_x.append(odom.pose.pose.position.x)
            pos_y.append(odom.pose.pose.position.y)
            goalx.append(goal[0])
            goaly.append(goal[1])

            yaw.append(yaw_)
            steer.append(float64.data)
            throttle.append(twist.linear.x)

            dict = {"time": time,
                    'x' : pos_x,
                    'y' : pos_y,
                    'goalx': goalx,
                    'goaly': goaly,
                    'yaw': yaw,
                    'steer': steer,
                    'throttle': throttle
                    }
            print(len(time))
            df = pd.DataFrame(dict)
            # saving the dataframe
            df.to_csv('{}.csv'.format(filename))
            rospy.sleep(1./5.)
        else:
            rospy.sleep(1./20.)
            print("waiting")