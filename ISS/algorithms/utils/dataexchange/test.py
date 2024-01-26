#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def generate_random_image(width, height):
    """
    生成一个随机的图像。
    """
    return np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

def publish_random_image():
    rospy.init_node('random_image_publisher', anonymous=True)
    pub = rospy.Publisher('/random_image', Image, queue_size=10)
    rate = rospy.Rate(1)  # 每秒发送一次
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # 生成随机图像
        random_image = generate_random_image(640, 480)

        # 将NumPy数组转换为ROS Image消息
        image_message = bridge.cv2_to_imgmsg(random_image, encoding="bgr8")

        # 发布消息
        pub.publish(image_message)
        rospy.loginfo("发布了一幅随机图像")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_image()
    except rospy.ROSInterruptException:
        pass
