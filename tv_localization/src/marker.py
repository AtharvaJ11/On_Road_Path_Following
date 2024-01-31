#!/usr/bin/env python3
# from re import T
# from telnetlib import Telnet
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped



# //only if using a MESH_RESOURCE marker type:
# marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
# vis_pub.publish( marker )
goal = PoseStamped()
def markerPoint_clbk(msg):
    global goal
    # print(msg)
    goal = msg

def main():
    global marker
    rospy.init_node("goalDikhado", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("goalVisualize", Marker, queue_size=1)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, markerPoint_clbk)


    marker = Marker()
    rospy.loginfo("Node begun")
    while rospy.is_shutdown is not True:
        marker.header.frame_id = "map"
        # marker.header.stamp = ros::Time();
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.pose.position.z = goal.pose.position.z
        marker.pose.orientation.x = goal.pose.orientation.x
        marker.pose.orientation.y = goal.pose.orientation.y
        marker.pose.orientation.z = goal.pose.orientation.z
        marker.pose.orientation.w = goal.pose.orientation.w
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 0.05
        marker.color.a = 0.5 # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        pub.publish(marker)
    # rospy.spin()
    pass

if __name__ == "__main__":
    main()