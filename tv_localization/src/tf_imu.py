# !/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


import tf
# import turtlesim.msg

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
                     #tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     (msg.pose.pose.orientation.x,
                     msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z,
                     msg.pose.pose.orientation.w,),
                     rospy.Time.now(),
                     "imu_link",
                     "map")


if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/uwb_odom',Odometry,handle_turtle_pose)
    rospy.spin()