#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial_comm import SerialComm


class JointStateSerialSubscriber(Node):
    def __init__(self):
        super().__init__('jss_subscriber')

        self.serial = SerialComm()
        self.serial.open_connection()

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.jss_callback,
            10
        )
        self.subscription

    def __del__(self):
        self.serial.close_connection()

    def jss_callback(self, msg):
        # self.get_logger().info('Received Joint State:')
        # self.get_logger().info(f'Names: {msg.name}')
        # self.get_logger().info(f'Positions: {msg.position}')
        # self.get_logger().info(f'Velocities: {msg.velocity}')
        # self.get_logger().info(f'Efforts: {msg.effort}')

        # self.get_logger().info('Serial Output')
        serial_msg = f'j1{round(1000*msg.position[0])}j2{round(msg.position[1])}j3{round(msg.position[2])}j4{round(msg.position[3])}j5{round(msg.position[4])}'

        # self.get_logger().info(serial_msg)
        self.serial.send_message(serial_msg)


def main(args=None):
    rclpy.init(args=args)

    jss_subscriber = JointStateSerialSubscriber()

    rclpy.spin(jss_subscriber)

    jss_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
