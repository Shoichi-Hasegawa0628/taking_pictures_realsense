#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import cv_bridge




class EnterCommand():
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.rgb_image = None
        self.cv_rgb_image = None
        self.bridge = cv_bridge.CvBridge()
        self._error = False
        # self.pub_object_name = rospy.Publisher("/object_name", String, queue_size=10)
        # self.pub_command = rospy.Publisher("/command", String, queue_size=10)

    def StartPublish(self): 
        step = 0
        object_name = input("Please input object name : \n")
	
        while step != 30:
            key = input("Please input Enter key : \n")
            print('The number of capture:{}\n'.format(str(step)))

            try:
                rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10.0)
                cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")

                # make folder
                DATA_PATH = "/root/HSR/catkin_ws/src/taking_pictures_realsense/{}".format(object_name)
                if not os.path.exists(DATA_PATH):
                    os.makedirs(DATA_PATH)

                cv2.imwrite(DATA_PATH + "/rgb_{}.png".format(str(step)), cv_rgb_image)

            except Exception as e:
                rospy.logerr('An error occurred when retrieving the RGB/depth images and camera parameters.')
                rospy.logerr(type(e))
                rospy.logerr(e.args)
                rospy.logerr(e)

            # for t in range(0, 8, 2):
            #     self.pub_object_name.publish(str(object_name))
            #     time.sleep(0.5)
            # for t in range(0, 8, 2):
            #     self.pub_command.publish(str(step))
            #     time.sleep(0.5)

            step += 1


if __name__ == '__main__':
    rospy.init_node('enter_command', anonymous=False)
    enter = EnterCommand()
    enter.StartPublish()
    # rospy.spin()
