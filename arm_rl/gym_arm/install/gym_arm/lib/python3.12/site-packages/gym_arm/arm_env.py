#!/usr/bin/env python3

import gym
import rclpy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import subprocess

def run_launch_file(package_name, launch_file_name):
    try:
        command = ["ros2", "launch", package_name, launch_file_name]
        subprocess.run(command, check=True)
        print(f"Launch file '{launch_file_name}' from package '{package_name}' executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing launch file: {e}")
    except FileNotFoundError:
        print("ros2 command not found. Is ROS2 sourced?")

class ArmEnv(gym.Env):
    def __init__(self):
        # TODO: init ROS node

        run_launch_file("arm_rl", "rviz_connection.launch.py")

        # ROS pub to send joint commands
        self.joint_pub = rclpy.Publisher("/joint_state", JointState, queue_size=1)
        self.joint_names = ["J1", "J2", "J3", "J4", "J5"]

        # ROS subs to recieve joint states
        rclpy.Subscriber("/ee_pos", Pose, self.ee_pose_callback)

        
        # arm state variables
        self.num_joints = 6
        self.joint_pos = np.zeros(self.num_joints)
        self.joint_vel = np.zeros(self.num_joints)
        self.ee_pos = np.zeros(self.num_joints)

        # action and observation space
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self.num_joints,), dtype=np.float32
        )

        obs_low = np.array([-3.14] * self.num_joints + [-5] * self.num_joints + [-2, -2, 0])
        obs_high = np.array([3.14] * self.num_joints + [5] * self.num_joints + [2, 2, 2])
        self.observation_space = gym.spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

        self.target_position = np.array([0.5, 0.3, 0.5])

        # wait for ROS startup messages
        rclpy.sleep(1)

    def joint_state_callback(self, msg):
        self.joint_pos = np.array(msg.position)
        self.joint_vel = np.array(msg.velocity)

    def ee_pose_callback(self, msg):
        self.ee_pos = np.array([msg.position.x, msg.position.y, msg.position.z])

    def step(self, action):
        joint_cmd = JointState()
        joint_cmd.name = self.joint_names
        joint_cmd.position = np.clip(self.joint_pos + action * 0.1, -3.14, 3.14)
        self.joint_pub.publish(joint_cmd)

        # allow sim to update
        rclpy.sleep(0.1)

        # retrieve state
        obs = np.concatenate([self.joint_positions, self.joint_velocities, self.ee_pos])

        # calculate reward
        distance = np.linalg.norm(self.ee_pos - self.target_position)
        reward = -distance
        
        done = distance < 0.05

        return obs, reward, done, {}

        def reset(self):
            joint_cmd = Float64MultiArray()
            joint_cmd.data = np.zeros(self.num_joints)
            self.joint_pub.publish(joint_cmd)
            rclpy.sleep(1)

            return np.concatenate([self.joint_pos, self.joint_vel, self.ee_pos])

    def render(self, mode="human"):
        pass

    def close(self):
        pass

