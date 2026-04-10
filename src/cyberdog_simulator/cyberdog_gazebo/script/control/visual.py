#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # 订阅图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',   # 这里后面按你的实际topic改
            self.image_callback,
            10
        )

        # 发布黄线检测结果
        self.yellow_line_pub = self.create_publisher(Bool, '/is_yellow_line', 10)
        # 可选：发布黄色面积占比，方便调参
        self.yellow_ratio_pub = self.create_publisher(Float32, '/yellow_ratio', 10)
        self.bridge = CvBridge()
        self.frame = None

        self.get_logger().info("VisionNode started.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame = frame
            self.detect_yellow_line(frame)
        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    def detect_yellow_line(self, frame):
        """
        检测地面黄线，并发布 /is_yellow_line
        """
        h, w, _ = frame.shape

        # 只看图像下半部分中间区域，避免远处和两侧干扰
        roi = frame[int(h * 0.65):h, int(w * 0.25):int(w * 0.75)]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 黄色阈值，后面要按仿真实际画面调
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([40, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        yellow_pixels = np.sum(mask > 0)
        total_pixels = mask.shape[0] * mask.shape[1]
        yellow_ratio = yellow_pixels / total_pixels if total_pixels > 0 else 0.0

        # 阈值先给一个初始值，后面调试
        is_yellow = yellow_ratio > 0.08

        # 发布 Bool
        yellow_msg = Bool()
        yellow_msg.data = is_yellow
        self.yellow_line_pub.publish(yellow_msg)

        # 发布 ratio，方便调试
        ratio_msg = Float32()
        ratio_msg.data = float(yellow_ratio)
        self.yellow_ratio_pub.publish(ratio_msg)

        # 调试信息
        self.get_logger().info(
            f'yellow_ratio={yellow_ratio:.3f}, is_yellow={is_yellow}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
