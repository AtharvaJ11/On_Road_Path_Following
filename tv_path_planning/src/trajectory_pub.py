from collections import deque
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String, Bool
import rospy
from geometry_msgs.msg import PoseStamped
import collections
import math


class Trajectory:
    def __init__(self):
        self._buffer_size = 5
        self._waypoints_queue = collections.deque(maxlen=20000)
        self._path_queue = collections.deque(maxlen=20000)
        self._waypoint_buffer = collections.deque(maxlen=self._buffer_size)

        self._current_pose = None
        self._current_speed = None
        self._target_pose = PoseStamped()

        self._odometry_subscriber = rospy.Subscriber("uwb_odom_positionOnly", Odometry, self.odometry_cb)
        self._path_subscriber = rospy.Subscriber("path", Path, self.path_cb)
        self._goal_publisher = rospy.Publisher("next_goal", PoseStamped, queue_size=10)
    
        self._path_queue = self._waypoints_queue
        print(self._path_queue)
    def odometry_cb(self, odometry_msg):
        self._current_pose = odometry_msg.pose.pose
        self.run_step()

    def path_cb(self, path_msg):
        # self._waypoint_buffer.clear()
        # self._waypoints_queue.clear()
        self._waypoints_queue.extend([pose.pose for pose in path_msg.poses])

    def run_step(self):
        if not self._waypoint_buffer and not self._waypoints_queue:
            rospy.loginfo("Waiting for a route...")
            return
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(self._waypoints_queue.popleft())
                else:
                    break


        self._target_pose.pose = self._waypoint_buffer[0]
        # print(target_pose)
        self._goal_publisher.publish(self._target_pose)

        max_index = -1

        min_distance = 2
        for i, route_point in enumerate(self._waypoint_buffer):
            print(route_point)
            if distance_vehicle(route_point, self._current_pose.position) < min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        # self._target_pose.pose = self._path_queue[0]
        # # print(target_pose)
        # self._goal_publisher.publish(self._target_pose)

        # max_index = -1

        # min_distance = 2
        # for i, route_point in enumerate(self._path_queue):
        #     # print(route_point)
        #     if distance_vehicle(route_point, self._current_pose.position) < min_distance:
        #         max_index = i
        # if max_index >= 0:
        #     for i in range(max_index + 1):
        #         self._path_queue.popleft()



def distance_vehicle(waypoint, vehicle_position):
    """
    calculate distance between waypoint and vehicle position
    """
    dx = waypoint.position.x - vehicle_position.x
    dy = waypoint.position.y - vehicle_position.y
    return math.sqrt(dx * dx + dy * dy)


def main(args=None):
    rospy.init_node("trajectory_pub")
 
    trajectory = Trajectory()
    rospy.spin()
if __name__ == "__main__":
    main()