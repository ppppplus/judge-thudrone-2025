#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse

def image_publisher(name):
    rospy.init_node("image_publisher", anonymous=True)

    # 创建发布者
    pub = rospy.Publisher("/image", Image, queue_size=10)
    rate = rospy.Rate(10)  # 发布频率 10Hz

    bridge = CvBridge()

    # 读取本地图片
    img_path = os.path.join(os.path.dirname(__file__), name)
    if not os.path.exists(img_path):
        rospy.logerr("图片文件不存在: %s" % img_path)
        return

    cv_image = cv2.imread(img_path)
    if cv_image is None:
        rospy.logerr("无法读取图片: %s" % img_path)
        return

    rospy.loginfo("开始发布图片: %s" % img_path)

    while not rospy.is_shutdown():
        try:
            # 转换为 ROS Image 消息
            ros_img = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            pub.publish(ros_img)
        except Exception as e:
            rospy.logerr("发布图片失败: %s" % str(e))

        rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, default="test.png")
    args = parser.parse_args()
    print(args.name)
    try:
        image_publisher(args.name)
    except rospy.ROSInterruptException:
        pass
