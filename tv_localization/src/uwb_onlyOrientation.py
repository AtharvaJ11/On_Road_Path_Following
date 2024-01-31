#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rospy
from time import time
from pypozyx import *
from tf.transformations import *

pub = rospy.Publisher('uwb_odom_orientationOnly', Odometry, queue_size=10)    
rospy.init_node('uwb_onlyOrientation', anonymous=True, disable_signals=True)

serial_port = get_first_pozyx_serial_port()
if serial_port is not None:
    pozyx_usb = PozyxSerial(serial_port)
    rospy.loginfo("Connection success!")
else:
    rospy.loginfo("No Pozyx port was found")
    while(serial_port is None):
        # rospy.sleep(0.1)
        rospy.loginfo("Finding Pozyx")
        serial_port = get_first_pozyx_serial_port()
    else:
        rospy.sleep(0.3)
        pozyx_usb = PozyxSerial(serial_port)
        rospy.loginfo("Connection Success at last!")

tag_pose = Odometry()
sensor_data = SensorData()
calibration_status = SingleRegister()

while rospy.is_shutdown is not True:
    status = pozyx_usb.getAllSensorData(sensor_data, None)
    status &= pozyx_usb.getCalibrationStatus(calibration_status, None)
    if status == POZYX_SUCCESS:
        quat = sensor_data.quaternion
        print(quat)
    else:
        #reconnect
        rospy.sleep(0.2)
        serial_port = get_first_pozyx_serial_port()
        if serial_port is not None:
            print(serial_port)
            try:
                pozyx_usb = PozyxSerial(serial_port)
            except:
                serial_port is None
                pass
            rospy.loginfo("Connection success!")
        else:
            rospy.loginfo("No Pozyx port was found")
            while(serial_port is None):
                # rospy.sleep(0.1)
                rospy.loginfo("Finding Pozyx")
                serial_port = get_first_pozyx_serial_port()
            else:
                rospy.sleep(0.1)
                pozyx_usb = PozyxSerial(serial_port)
                rospy.loginfo("Connection Success at last!")

    euler_angles = EulerAngles()
    pozyx_usb.getEulerAngles_deg(euler_angles)

    tag_pose.header.frame_id = 'map'
    tag_pose.pose.pose.position.x =  0
    tag_pose.pose.pose.position.y =  0
    tag_pose.pose.pose.position.z =  0

    tag_pose.pose.pose.orientation.x = -quat[1]/16384.
    tag_pose.pose.pose.orientation.y = -quat[2]/16384.
    tag_pose.pose.pose.orientation.z = quat[3]/16384.
    tag_pose.pose.pose.orientation.w = quat[0]/16384.

    #//////IC engine lab yaw correction////////////////
    lab_magnetYaw_correction = True
    if lab_magnetYaw_correction is True:
        q_org = [tag_pose.pose.pose.orientation.x,
                    tag_pose.pose.pose.orientation.y,
                    tag_pose.pose.pose.orientation.z,
                    tag_pose.pose.pose.orientation.w]
        # q_rot = quaternion_from_euler(0, 0, 2.775043127747267 + 3.14159265359)
        q_rot = quaternion_from_euler(0, 0, 1.44164938+ 3.14159265359)

        q_new = quaternion_multiply(q_rot, q_org)

        tag_pose.pose.pose.orientation.x = q_new[0]
        tag_pose.pose.pose.orientation.y = q_new[1]
        tag_pose.pose.pose.orientation.z = q_new[2]
        tag_pose.pose.pose.orientation.w = q_new[3]
    #///////////done on 12 may 2022////////////////////

    #////////////tag position transformation///////////
    tag_posOnVehicle_tf = True
    if tag_posOnVehicle_tf is True:
        quaternion = (
            tag_pose.pose.pose.orientation.x,
            tag_pose.pose.pose.orientation.y,
            tag_pose.pose.pose.orientation.z,
            tag_pose.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw_ = euler[2]

        l = 1.2679
        x_tf = l * math.cos(yaw_)
        y_tf = l * math.sin(yaw_)
        z_tf = 0
        # print (x_tf,y_tf, z_tf)

        tag_pose.pose.pose.position.x +=  x_tf
        tag_pose.pose.pose.position.y +=  y_tf
        tag_pose.pose.pose.position.z +=  z_tf
    #//////////////////////////////////////////////////


    # q.normalize()
    pub.publish(tag_pose)
    rate = rospy.Rate(10) # 10hz