#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern


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
        rospy.sleep(1)
        self.pub.publish(msg)

        rospy.sleep()

        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0
        msg.vel_right = 0
        rospy.sleep(1)
        self.pub.publish(msg)

    def run(self):
        rospy.wait_for_service(
            '/duckiebotX/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy(
                '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = ['white', '', '', '', '']
            msg.color_mask = [1, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        rospy.sleep(1)
        drive(0.5)
        rospy.sleep(10)

        rospy.wait_for_service(
            '/duckiebotX/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy(
                '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = ['yellow', '', '', '', '']
            msg.color_mask = [1, 0, 0, 0, 0]
            msg.frequency = 1
            msg.frequency_mask = [1, 0, 0, 0, 0]
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        rospy.sleep(1)
        turn(5)

        rospy.wait_for_service(
            '/duckiebotX/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy(
                '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = ['white', '', '', '', '']
            msg.color_mask = [1, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        rospy.sleep(1)
        drive(0.5)
        rospy.sleep(10)
        drive(0)
        rospy.wait_for_service(
            '/duckiebotX/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy(
                '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = ['white', 'red', '', '', '']
            msg.color_mask = [0, 1, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
