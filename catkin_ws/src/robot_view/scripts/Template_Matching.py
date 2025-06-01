#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import time
from sensor_msgs.msg import Image

def call_back(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        cv2.imshow("src", cv_image)
        cv2.waitKey(1)
        
        # template matching
        template = cv2.imread('/home/robot/catkin_ws/src/robot_view/template.bmp')
        if template is None:
            rospy.logerr("无法加载模板图像")
            return
            
        cv2.imshow("template", template)
        cv2.waitKey(2)
        
        h, w = template.shape[:2]
        
        res = cv2.matchTemplate(cv_image, template, cv2.TM_SQDIFF)
        
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        
        top_left = min_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(cv_image, top_left, bottom_right, (0, 0, 255), 2)
        
        cv2.imshow("dst", cv_image)
        cv2.waitKey(1)
    
    except Exception as e:  # 修复异常处理
        print(e)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('template_matching', anonymous=False)
    
    # 机器人摄像头话题
    img_topic = '/camera/rgb/image_raw'
    # 电脑摄像头
    # img_topic = '/usb_cam/image_raw'
    # 创建订阅器话题
    image_sub = rospy.Subscriber(img_topic, Image, call_back)
    
    # 保持节点运行
    rospy.loginfo("模板匹配节点已启动，等待图像...")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("关闭中...")
    
    # 关闭所有窗口
    cv2.destroyAllWindows()