#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped


class MyNode(DTROS):
    def __init__(self,  node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub = self.publisher('/duckiebot2/wheels_driver_node/wheels_cmd',
                                  WheelsCmdStamped,
                                  queue_size=1)

    def drive(self, speed):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = speed
        msg.vel_right = speed
        rospy.sleep(1)
        self.pub.publish(msg)
    
    def turn(self, time):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = -0.3
        msg.vel_right = 0.3
        rospy.sleep(time)
        self.pub.publish(msg)

        rospy.sleep();

        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0
        msg.vel_right = 0
        rospy.sleep(1)
        self.pub.publish(msg)

    def run(self):
        drive(0.5)
        rospy.sleep(10)
        turn(5)
        drive(0.5)
        rospy.sleep(10)
        drive(0);


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
