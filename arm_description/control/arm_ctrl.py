#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np

class ArmCtrl(Node):
    def __init__(self):
        super().__init__('arm_ctrl')

        self.ee_pose_pub = self.create_publisher(Pose, '/ee_pose', 10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.ee_callback)

    def ee_callback(self):
        msg = Pose()
        # TOD0: forward kinematics of end-effector
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        self.ee_pose_pub.publish(msg)







def main(args=None):
    rclpy.init(args=args)

    arm_ctrl = ArmCtrl()

    rclpy.spin(arm_ctrl)

    arm_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
