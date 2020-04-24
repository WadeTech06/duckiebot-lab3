#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern


class MyNode(DTROS):
    def __init__(self,  node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.sub = self.subscriber('/duckiebotX/camera_node/image/compressed',CompressedImage,self.onImageReceived)

    def detectRedObject(self, img):
        # set red range and mask of red colored objects
        lower_red = np.array([17, 15, 100])
        upper_red = np.array([50, 56, 200]) 

        red_mask = cv2.inRange(img,lower_red,upper_red)

        result = cv2.bitwise_and(img,img, mask= red_mask)
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        count = cv2.countNonZero(gray)

        if count > 0:
            rospy.wait_for_service('/duckiebotX/led_emitter_node/set_custom_pattern')
            try:
                service = rospy.ServiceProxy(
                    '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
                msg = LEDPattern()
                msg.color_list = ['', 'red', '', '', '']
                msg.color_mask = [0, 1, 0, 0, 0]
                msg.frequency = 0
                msg.frequency_mask = [0, 0, 0, 0, 0]
                response = service(msg)
                rospy.loginfo(response)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        else:
            rospy.wait_for_service('/duckiebotX/led_emitter_node/set_custom_pattern')
            try:
                service = rospy.ServiceProxy(
                    '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
                msg = LEDPattern()
                msg.color_list = ['', 'red', '', '', '']
                msg.color_mask = [0, 0, 0, 0, 0]
                msg.frequency = 0
                msg.frequency_mask = [0, 0, 0, 0, 0]
                response = service(msg)
                rospy.loginfo(response)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e   

    def detectYellowObject(self,img):
        # set yellow range and mask of yellow colored objects
        lower_yellow = np.array([0, 153, 153])
        upper_yellow = np.array([102, 255, 255]) 

        yellow_mask = cv2.inRange(img,lower_yellow,upper_yellow)

        result = cv2.bitwise_and(img,img, mask= yellow_mask)
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        # get the image dimensions
        r = gray.shape[0]
        c = gray.shape[1]

        rightlimit = int(c*.6)
        count=0
        # loop over the image, pixel by pixel
        for y in range(0, r):
            for x in range(rightlimit, c):
                # threshold the pixel
                if gray[y, x] > 0:
                    count +=1
                    
        if count > 0:
            rospy.wait_for_service('/duckiebotX/led_emitter_node/set_custom_pattern')
            try:
                service = rospy.ServiceProxy(
                    '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
                msg = LEDPattern()
                msg.color_list = ['yellow', '', '', '', '']
                msg.color_mask = [1, 0, 0, 0, 0]
                msg.frequency = 0
                msg.frequency_mask = [0, 0, 0, 0, 0]
                response = service(msg)
                rospy.loginfo(response)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        else:
            rospy.wait_for_service('/duckiebotX/led_emitter_node/set_custom_pattern')
            try:
                service = rospy.ServiceProxy(
                    '/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
                msg = LEDPattern()
                msg.color_list = ['yellow', '', '', '', '']
                msg.color_mask = [1, 0, 0, 0, 0]
                msg.frequency = 0
                msg.frequency_mask = [0, 0, 0, 0, 0]
                response = service(msg)
                rospy.loginfo(response)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e                    

    def onImageReceived(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        detectRedObject(image_np)
        detectYellowObject(image_np)


         

        # visualize image 
        #image_message = CvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    def run(self):
         rospy.spin()


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
