#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped
# from geometry_msgs.msg import Twist
from tf import transformations
import math

# position_ = Odometry()
position_ = Point()
yaw_ = 0.1
throttle_scale = 320 / 400 # to be published as cmd_vel

def odom_clbk(msg):
    global position_
    position_ = msg

def change_state(state):
    global state_
    state_ = state

    twist_msg = Twist()
    if state == 2:
        twist_msg.linear.x = 0
    pass

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    # print('position_: ', position_)
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

dist_precision_ = 1 #meters

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_prescision_, state_, dist_precision_

    # print('position_ 2: ', position_)
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - (yaw_ - math.pi/2)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    if err_pos > dist_precision_:
        twist_msg = Twist()
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = 0
        twist_msg.linear.x = throttle_scale#0.4
        # twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        print('Position error: [%s]' % err_pos)
        print("Yaw: {}".format(yaw_))
        # print(throttle_scale, last_published_throttle_value)
        pub.publish(twist_msg)
    else:
        print('Position error: [%s]' % err_pos)
        change_state(2)
        
        # # state change conditions
        # if math.fabs(err_yaw) > yaw_precision_:
        #     # print('Yaw error: [%s]' % err_yaw)
        #     if state_ != 2:
        #         change_state(0)

def moveLikeJagger(des_pos):
    global yaw_, state_
    state_ = 1
    done()
    print('Desired position MLJ: [%s, %s]' % (des_pos.x, des_pos.y))
    rospy.logwarn("Vehicle is heading to goal \n{}".format(des_pos))
    while(state_ != 2): #Here was the logical error - replaced if with while
        rospy.sleep(0.2)
        # print("Yaw: {}", yaw_)
        # State_Msg = 'State: [%s]' % state_
        # rospy.loginfo(State_Msg)
        if state_ == 0:
            # fix_yaw(des_pos)
            pass
        elif state_ == 1:
            goal_pub.publish(goal_position)

            go_straight_ahead(des_pos)
        else:
            rospy.logerr('Unknown state!')
    else:
        done()

def done():
    print('Soup Song')
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    for _ in range(10):
        rospy.sleep(0.05)
        pub.publish(twist_msg)

if __name__ == "__main__":

    rospy.init_node("testPointFollower",anonymous=True, disable_signals=True)
    rospy.Subscriber("uwb_odom_low_pass", Odometry, clbk_odom)
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    
    # goals = [[-25.7,15.5]]
    goals = [[-25.7,15.5], [-34.2,12.6]]

    for goal in goals:

        state_ = 1
        position = Point()
        # position1.x = -25.7
        # position1.y = 15.5
        position.x = goal[0]
        position.y = goal[1]
        position.z = 0

        goal_position = PoseStamped()
        goal_position.pose.position.x = position.x
        goal_position.pose.position.y = position.y
        goal_position.pose.position.z = position.z

        goal_position.pose.orientation.x = 0
        goal_position.pose.orientation.y = 0
        goal_position.pose.orientation.z = 0
        goal_position.pose.orientation.w = 1

        for _ in range(20):
            goal_pub.publish(goal_position)
            # done()
        #     rospy.sleep(0.1)
        # state_ = 1
        moveLikeJagger(position)


    
    goal_position.header.frame_id = 'Now stop'
    goal_position.pose.position.y = 0
    goal_position.pose.position.x = 0
    goal_position.pose.position.z = 0

    goal_position.pose.orientation.x = 0
    goal_position.pose.orientation.y = 0
    goal_position.pose.orientation.z = 0
    goal_position.pose.orientation.w = 1
    
    goal_pub.publish(goal_position)


    rospy.spin()
       # position2 = Point()
    # position2.x = -17.365
    # position2.y = 6.146
    # position2.z = 0

    # goal_position.pose.position.x = position2.x
    # goal_position.pose.position.y = position2.y
    # goal_position.pose.position.z = position2.z

    # moveLikeJagger(position2)

    # for _ in range(30):
    #     goal_pub.publish(goal_position)
    #     # done()
    #     rospy.sleep(0.1)

    pass