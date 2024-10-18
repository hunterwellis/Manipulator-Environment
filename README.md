# Robot Arm Manipulator Environment

ROS2 packages for training my 6-axis robot arm (open-source hardware design by tw2ka) for reinforcement learning tasks.

Using ROS2 Jazzy, Jazzy control packages, and Gazebo Harmonic. 

![arm rviz](./images/arm_rviz.png)

*Arm model in RViz.*

## arm_description
Contains robot URDF, meshes, and gazebo launch files

build the package like any other ROS package &ndash; from the package dir:
```
colcon build
```
source the package,
```
source install/setup.bash
```
launch the arm in the gazebo environment
```
ros2 launch arm_description gazebo.launch.py
```

### TODO:
- inverse kinematics for end effector manipulation
- add objects for manipulation
- add camera sensor
