#!/usr/bin/env python3

import os
import time
import subprocess
import rclpy
from rclpy.node import Node
from pathlib import Path
import psutil


WORLD_LIST = [
    'world_0',
    'world_1',
    'world_2',
    'world_3',
    'world_4',
    'world_5',
    'world_6',
    'world_7',
    'world_8',
    'world_9',
    'world_10',
    'world_11',
    'world_12',
    'world_13',
    'world_14',
    'world_15',
    'world_16',
    'world_17',
    'world_18',
    'world_19',
    'world_20',
    'world_21',
    'world_22',
    'world_23',
    'world_24',
    'world_25',
    'world_26',
    'world_27',
    'world_28',
    'world_29',
    'world_30',
    'world_31',
    'world_32',
    'world_33',
    'world_34',
    'world_35',
    'world_36',
    'world_37',
    'world_38',
    'world_39',
    'world_40',
    'world_41',
    'world_42',
    'world_43',
    'world_44',
    'world_45',
    'world_46',
    'world_47',
    'world_48',
    'world_49',
    'world_50',
    'world_51',
    'world_52',
    'world_53',
    'world_54',
    'world_55',
    'world_56',
    'world_57',
    'world_58',
    'world_59',
    'world_60',
    'world_61',
    'world_62',
    'world_63',
    'world_64',
    'world_65',
    'world_66',
    'world_67',
    'world_68',
    'world_69',
    'world_70',
    'world_71',
    'world_72',
    'world_73',
    'world_74',
    'world_75',
    'world_76',
    'world_77',
    'world_78',
    'world_79',
    'world_80',
    'world_81',
    'world_82',
    'world_83',
    'world_84',
    'world_85',
    'world_86',
    'world_87',
    'world_88',
    'world_89',
    'world_90',
    'world_91',
    'world_92',
    'world_93',
    'world_94',
    'world_95',
    'world_96',
    'world_97',
    'world_98',
    'world_99'
]

ARM_SIMULATION_PACKAGE = 'arm_simulation'
LAUNCH_FILE = 'world.gazebo.launch.py'

IMAGE_SAVER_SCRIPT = 'picture_subscriber.py'  # Your image-saving node

def start_gazebo(world_path):
    return subprocess.Popen([
        'ros2', 'launch', 'arm_simulation', 'world.gazebo.launch.py',
        f'world:={world_path}.sdf'
    ])

def kill_process_tree(proc):
    try:
        parent = psutil.Process(proc.pid)
        children = parent.children(recursive=True)
        for child in children:
            child.terminate()
        _, alive = psutil.wait_procs(children, timeout=5)
        for child in alive:
            child.kill()
        parent.terminate()
        parent.wait(timeout=5)
    except psutil.NoSuchProcess:
        pass
    except psutil.TimeoutExpired:
        parent.kill()
    print("Cleaned up Gazebo and child processes.")

class WorldScreenshotTaker(Node):

    def __init__(self):
        super().__init__('world_screenshot_taker')

        self.declare_parameter("worlds", WORLD_LIST)

        self.worlds = self.get_parameter("worlds").get_parameter_value().string_array_value
        self.run_all_worlds()

    def run_all_worlds(self):
        for world_file in WORLD_LIST:
            self.get_logger().info(f"\n Launching world: {world_file}")

            # Start Gazebo simulation
            gazebo_process = start_gazebo(world_file)
            time.sleep(10)

            self.get_logger().info(f"Capturing image in world: {world_file}")
            image_process = subprocess.run(['ros2', 'run', ARM_SIMULATION_PACKAGE, IMAGE_SAVER_SCRIPT, '--ros-args', '-p', f'image_name:={world_file}'])

            self.get_logger().info(f"Image capture complete. Shutting down Gazebo.")

            kill_process_tree(gazebo_process)
            time.sleep(2)

            subprocess.run(['pkill', '-f', 'gz sim server'])
            subprocess.run(['pkill', '-f', 'gz sim client'])

            time.sleep(5)  # Optional delay between worlds

        self.get_logger().info("Finished capturing all screenshots.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = WorldScreenshotTaker()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
