#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class SingleImageSaver(Node):

    def __init__(self):
        super().__init__('single_image_saver')
        self.bridge = CvBridge()

        self.declare_parameter('image_name', 'world_default')
        self.image_name = self.get_parameter('image_name').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        self.image_saved = False
        self.get_logger().info("Waiting for image...")

    def image_callback(self, msg):
        if not self.image_saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = f"{self.image_name}.png"
                save_path = os.path.join(os.getcwd(),
                                         'src',
                                         'arm_simulation',
                                         'scripts',
                                         'images',
                                         filename)
                # print(save_path)
                cv2.imwrite(save_path, cv_image)
                self.get_logger().info(f"Image saved as {filename}")
                self.image_saved = True

                # Shutdown after saving
                rclpy.shutdown()

            except Exception as e:
                self.get_logger().error(f"Failed to save image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    image_saver = SingleImageSaver()
    rclpy.spin(image_saver)


if __name__ == '__main__':
    main()
