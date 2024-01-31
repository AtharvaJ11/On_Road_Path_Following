#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class killSwitch():
    def __init__(self):

        rospy.Subscriber("arduino1", String, self.throttle_cb)
        rospy.Subscriber("steer_arduino1", String, self.steer_cb)
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.steer_pub = rospy.Publisher('steer_arduino', String, queue_size=10)
        self.th_pub = rospy.Publisher('arduino', String, queue_size=10)
        self.canMove = False
        self.steerCentre = False

    def joy_cb(self,joy_msg):
        if(joy_msg.buttons[2]==1):
            self.canMove=True
        else:
            self.canMove=False
        if(joy_msg.buttons[1]==1):
            self.steerCentre = True
        else:
            self.steerCentre = False
        rospy.loginfo("Kill movement: {}".format(not self.canMove))

    def steer_cb(self,steer_msg):

  
        
        if self.steerCentre is True:
            self.steer_pub.publish("s20000")
        else:
            self.steer_pub.publish(steer_msg)

            #publish zero steer
            pass

    def throttle_cb(self,throttle_msg):
        if self.canMove is True:
            self.th_pub.publish(throttle_msg)
        else:
            self.th_pub.publish("t0")
            #publish zero throttlei
            pass


if __name__ == '__main__':
    rospy.init_node('kill_switch', anonymous=True)
    switch = killSwitch()
    rospy.spin()