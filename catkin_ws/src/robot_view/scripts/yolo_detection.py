#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # 订阅YOLO检测结果
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # 发布处理后的图像
        self.image_pub = rospy.Publisher('/yolo/detection_image', Image, queue_size=1)
        
        self.latest_detections = None
        self.latest_image = None
        
    def detection_callback(self, data):
        self.latest_detections = data.bounding_boxes
        for box in data.bounding_boxes:
            if box.Class == "drink_can":
                rospy.loginfo(f"检测到易拉罐: 置信度 {box.probability:.2f}, 位置 ({box.xmin}, {box.ymin})-({box.xmax}, {box.ymax})")
                
        # 如果有图像，绘制检测框
        if self.latest_image is not None and self.latest_detections:
            self.draw_detections()
    
    def image_callback(self, data):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换错误: {e}")
    
    def draw_detections(self):
        """在图像上绘制检测框"""
        if self.latest_image is None:
            return
            
        img = self.latest_image.copy()
        
        for box in self.latest_detections:
            if box.Class == "drink_can" and box.probability > 0.5:
                # 绘制边界框
                cv2.rectangle(img, (int(box.xmin), int(box.ymin)), 
                            (int(box.xmax), int(box.ymax)), (0, 255, 0), 2)
                
                # 添加标签
                label = f"易拉罐: {box.probability:.2f}"
                cv2.putText(img, label, (int(box.xmin), int(box.ymin-10)), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 发布处理后的图像
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"图像发布错误: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass