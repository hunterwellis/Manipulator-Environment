#!/usr/bin/env python3

import rclpy
from rlcpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSerialSubscriber(Node):
    def __init__(self):
        super().__init__('jss_subscriber')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.jss_callback,
            10
        )
        self.subscription

    def jss_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    jss_subscriber = JointStateSerialSubscriber()

    rclpy.spin(jss_subscriber)

    jss_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
