import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Path
f = open('jun28path1.json')

# Python program to read
# json file
 
 
import json
 
# # Opening JSON file
# f = open('pose.json')
#  # returns JSON object as
# # a dictionary
# data = json.load(f)
# # Closing file
# f.close()
 
# Iterating through the json
# list


# for i in data['pose']:
#     print(i)
# print((data['header']))
# print((data['pose']))


#create header
# header_data = data['header']
def headerFromJSON(header_data):
    header = Header()
    header.seq = header_data['seq']
    header.stamp.secs = header_data['stamp']['secs']
    header.stamp.nsecs = header_data['stamp']['nsecs']
    header.frame_id = header_data['frame_id']
    return header
# header = headerFromJSON(data['header'])



# pose_data = data['pose']
# position_data = pose_data['position']
# #create poses
def poseFromJSON(pose_data):
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
def poseStampedFromJSON(header,pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = poseFromJSON(pose)
    pose_stamped.header = headerFromJSON(header)
    return pose_stamped
# pose_stamped = poseStampedFromJSON(header, pose)


if __name__ == "__main__":
    # Opening JSON file
    f = open('jun28path3.json')

    # returns JSON object as
    # a dictionary
    data = json.load(f)
    # Closing file
    f.close()
    # for i in data:
    #     print(i)

    path = Path()
    path.header = headerFromJSON(data['header'])
    pose_stamped_array = []
    rospy.init_node('path_pub', anonymous=True, disable_signals=True)
    for _ in range(path.header.seq):
        pose_stamped = data['poses'][_]
        # print(pose_stamped)
        pose_stamped_array.append(poseStampedFromJSON(pose_stamped['header'],pose_stamped['pose']))
    path.poses = pose_stamped_array
    # print(pose_stamped)
    path_pub = rospy.Publisher('path', Path, queue_size=10)    
    while rospy.is_shutdown is not True:
        path_pub.publish(path)
    # path_pub.publish(path)