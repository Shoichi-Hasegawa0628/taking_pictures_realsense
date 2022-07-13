#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image as ImageMsg
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import cv_bridge
from PIL.Image import Image
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import time


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
        object_name = input("Please input place name : \n")

        while step !=5:
            # rgb_msg = None
            # cv_rgb_image = None
            # print(cv_rgb_image)
            key = input("Please input Enter key : \n")
            print('The number of capture:{}\n'.format(str(step)))

            try:
                self.rgb_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_raw", ImageMsg, timeout=None)
                rospy.loginfo("Get Image message")
                self.cv_rgb_image = self.bridge.imgmsg_to_cv2(self.rgb_msg, "bgr8")

                # make folder
                DATA_PATH = "/root/HSR/data/{}".format(object_name)
                if not os.path.exists(DATA_PATH):
                    os.makedirs(DATA_PATH)

                cv2.imwrite(DATA_PATH + "/{}.png".format(str(step)), self.cv_rgb_image)

            except Exception as e:
                rospy.logerr('An error occurred when retrieving the RGB/depth images and camera parameters.')
                rospy.logerr(type(e))
                rospy.logerr(e.args)
                rospy.logerr(e)

            step += 1


if __name__ == '__main__':
    rospy.init_node('enter_command', anonymous=False)
    enter = EnterCommand()
    enter.StartPublish()
    # rospy.spin()