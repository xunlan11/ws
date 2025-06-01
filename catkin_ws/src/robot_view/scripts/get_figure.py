#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import time
from sensor_msgs.msg import Image


def callback(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('take_photo', anonymous=False)
    img_topic = '/camera/rgb/image_raw'
    image_sub = rospy.Subscriber(img_topic, Image, callback)
    rospy.sleep(10)