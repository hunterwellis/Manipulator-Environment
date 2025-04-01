#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import time

class RandomJointPublisher(Node):

    def __init__(self):
        super().__init__('random_joint_publisher')

        # Publish to the /joint_states topic
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.005, self.publish_random_joints)

        # Define joint names (replace with your arm joint names)
        self.joint_names = [
            'J1',
            'J2',
            'J3',
            'J4',
            'J5'
        ]

        self.position = [random.uniform(-1.57, 1.57) for _ in self.joint_names]

    def publish_random_joints(self):
        # Create a JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names

        # Generate random positions within joint limits (-1.57 to 1.57 rad)
        for index, value in enumerate(self.position):
            self.position[index] = value + random.uniform(-0.001, 0.001)
        joint_state.position = self.position

        # Publish the random joint states
        self.publisher_.publish(joint_state)
        self.get_logger().info(f'Publishing random joint positions: {joint_state.position}')


def main(args=None):
    rclpy.init(args=args)
    random_joint_publisher = RandomJointPublisher()
    rclpy.spin(random_joint_publisher)
    random_joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
