#!/usr/bin/env python

import paho.mqtt.client as mqtt
import ssl
import json
host = "192.168.0.109" # fill in the IP of your gateway
port = 1883
topic = "tags" 
from nav_msgs.msg import Odometry
import rospy
# from time import time
from pypozyx import *
from statistics import variance
# imu = [0, 0, 0]

x = []
y = []
z = []

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
    global x,y,z

    if checkKey(loaded_json[0]['data'], 'coordinates'):


        tag_pose = Odometry()
        tag_pose.header.frame_id = 'map'
        tag_pose.pose.pose.position.x =  -loaded_json[0]['data']['coordinates']['x']/1000.0
        tag_pose.pose.pose.position.y =  loaded_json[0]['data']['coordinates']['y']/1000.0
        tag_pose.pose.pose.position.z =  loaded_json[0]['data']['coordinates']['z']/1000.0


        x.append(tag_pose.pose.pose.position.x)
        y.append(tag_pose.pose.pose.position.y)
        z.append(tag_pose.pose.pose.position.z)

        if(len(z) == 2000):
            rospy.loginfo("Variance in x : {}]\n".format(variance(x)))
            rospy.loginfo("Variance in y : {}]\n".format(variance(y)))
            rospy.loginfo("Variance in z : {}]\n".format(variance(z)))


        # rospy.loginfo("Published\n{}".format(tag_pose.pose.pose))
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
