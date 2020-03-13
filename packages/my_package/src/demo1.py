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

    def run(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0.1
        msg.vel_right = 0.1
        rospy.sleep(1)
        self.pub.publish(msg)


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()