#!/usr/bin/env python

import paho.mqtt.client as mqtt
import ssl
import json
host = "192.168.0.108" # fill in the IP of your gateway
port = 1883
topic = "tags" 
from nav_msgs.msg import Odometry
import rospy
# from time import time
from pypozyx import *

# imu = [0, 0, 0]

def on_connect(client, userdata, flags, rc):
    print(mqtt.connack_string(rc))
    client.subscribe(topic)

def checkKey(dict, key):
    if key in dict.keys():
        return True
    else:
        return False

# def imu_cb(data):
#     global imu
#     imu[0] = data.vector.x
#     imu[1] = data.vector.y
#     imu[2] = data.vector.z 

# callback triggered by a new Pozyx data packet
def on_message(client, userdata, msg):
    # print("Positioning update:", msg.payload.decode())
    loaded_json = json.loads(msg.payload.decode())

    if checkKey(loaded_json[0]['data'], 'coordinates'):


        tag_pose = Odometry()
        tag_pose.header.frame_id = 'map'
        tag_pose.pose.pose.position.x =  -loaded_json[0]['data']['coordinates']['x']/1000.0
        tag_pose.pose.pose.position.y =  loaded_json[0]['data']['coordinates']['y']/1000.0
        tag_pose.pose.pose.position.z =  loaded_json[0]['data']['coordinates']['z']/1000.0
        # sensor_data = SensorData()
        # calibration_status = SingleRegister()

        # status = pozyx_usb.getAllSensorData(sensor_data, None)
        # status &= pozyx.getCalibrationStatus(calibration_status, None)
        '''
        # if status == POZYX_SUCCESS:
        #     # publishSensorData(sensor_data, calibration_status)
        #     # print((sensor_data.quaternion))
        #     quat = sensor_data.quaternion
        #     tag_pose.pose.pose.orientation.x = -quat[1]/16384.
        #     tag_pose.pose.pose.orientation.y = -quat[2]/16384.
        #     tag_pose.pose.pose.orientation.z = quat[3]/16384.
        #     tag_pose.pose.pose.orientation.w = quat[0]/16384.

        #     #//////IC engine lab yaw correction////////////////
        #     lab_magnetYaw_correction = False
        #     if lab_magnetYaw_correction is True:
        #         q_org = [tag_pose.pose.pose.orientation.x,
        #                     tag_pose.pose.pose.orientation.y,
        #                     tag_pose.pose.pose.orientation.z,
        #                     tag_pose.pose.pose.orientation.w]
        #         q_rot = quaternion_from_euler(0, 0, 2.775043127747267 + 3.14159265359)
        #         q_new = quaternion_multiply(q_rot, q_org)

        #         tag_pose.pose.pose.orientation.x = q_new[0]
        #         tag_pose.pose.pose.orientation.y = q_new[1]
        #         tag_pose.pose.pose.orientation.z = q_new[2]
        #         tag_pose.pose.pose.orientation.w = q_new[3]
        #     #///////////done on 12 may 2022////////////////////

        #     #////////////tag position transformation///////////
        #     tag_posOnVehicle_tf = False
        #     if tag_posOnVehicle_tf is True:
        #         quaternion = (
        #             tag_pose.pose.pose.orientation.x,
        #             tag_pose.pose.pose.orientation.y,
        #             tag_pose.pose.pose.orientation.z,
        #             tag_pose.pose.pose.orientation.w)
        #         euler = euler_from_quaternion(quaternion)
        #         yaw_ = euler[2]

        #         l = 1.2679
        #         x_tf = l * math.cos(yaw_)
        #         y_tf = l * math.sin(yaw_)
        #         z_tf = 0
        #         # print (x_tf,y_tf, z_tf)

        #         tag_pose.pose.pose.position.x +=  x_tf
        #         tag_pose.pose.pose.position.y +=  y_tf
        #         tag_pose.pose.pose.position.z +=  z_tf
        #     #//////////////////////////////////////////////////
        # print("test")
        '''
        rospy.loginfo("Published\n{}".format(tag_pose.pose.pose))
        pub.publish(tag_pose)
    # rate.sleep()



def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed to topic!")

client = mqtt.Client()
# set callbacks
# serial_port = get_first_pozyx_serial_port()

# if serial_port is not None:
#     pozyx_usb = PozyxSerial(serial_port)
#     rospy.loginfo("Connection success!")
# else:
#     rospy.loginfo("No Pozyx port was found")
pub = rospy.Publisher('uwb_odom_positionOnly', Odometry, queue_size=10)    
rospy.init_node('uwb_onlyPosition', anonymous=True, disable_signals=True)
rate = rospy.Rate(10) # 10hz

client.connect(host, port=port)
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe

# works blocking, other, non-blocking, clients are available too.
client.loop_forever()
