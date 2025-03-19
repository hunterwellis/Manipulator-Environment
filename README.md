# Robot Arm Manipulator Environment

ROS2 packages for simulating and training a 6-axis robot arm (modified version of an [open-source hardware design by tw2ka](https://www.dropbox.com/scl/fi/mgowac0a7bwx7u2pcz12b/Arduino-robot-arm-files-step.zip?rlkey=3cpy6x4wcpfr1s548s7qxxex5&e=2&dl=0)) for reinforcement learning tasks.

![arm rviz](./images/jsp_demo.gif)
### Requirements
- Ubuntu 24.04 (Noble Numbat)
- ROS2 Jazzy
- Jazzy control packages
- Jazzy USB cam package
- Gazebo Harmonic

## Usage
To visualize the arm in RViz with the joint state publisher GUI.
```
ros2 launch arm_description rviz_jsp.launch.py
```
![arm rviz](./images/arm_rviz.png)

## Packages
**arm_description**
 -- Contains robot URDF, meshes, and launch files for RViz and the robot state publisher.

**arm_simulation**
-- The simulation package contains the SDF world files and other files related to setup of the Gazebo simulation environment.

**arm_rl** 
-- The reinforcement learning package has launch files to set up gym training environments and scripts to run RL algorithms.

**arm_cv** 
-- Computer vision package using YOLOv8 (with oriented bounding boxes). 

**arm_nlp** 
-- Natural language processing package for converting natural language prompts to symbolic representations.

**arm_link**
-- Serial communication link to arm.
