# robotiq_2f_gripper_ros2

A ROS2 package for use with the Robotiq 2-Finger 140mm gripper.

## Overview

The repo contains four ROS2 packages (detailed description below):
| Package Name | Description |
| --- | --- |
| robotiq_2f_gripper_description | A description package containing the URDF and meshes of the gripper to be viewed in RViz  |
| robotiq_2f_gripper_hardware | A hardware package creating a ROS2 action server to control the gripper |
| robotiq_2f_gripper_interfaces | A package defining the driver for the gripper |
| robotiq_2f_gripper_msgs | A packages defining a message template for the gripper communication  |

## Getting Started

These instrcutions assume you have ROS2 (tested with ROS2 Humble) installed.

Clone the repo to your ROS2 workspace and build the ROS2 packages:

```bash
colcon build
```

Don't forget to source your workspace after the built:

```bash
source install/setup.bash
```

## Detailed Package Descriptions

### robotiq_2f_gripper_description

Adopted from [ros-industrial](https://github.com/ros-industrial/robotiq) for ROS2. The original license remains active.

To launch RViz and to inspect the gripper run:

```bash
ros2 launch robotiq_2f_gripper_description visualize.launch
```

### robotiq_2f_gripper_hardware
 
To launch the ROS2 action server (this will initialize/move the gripper) run:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py
```

To send a command to the gripper run (remeber to source any new terminal you open with `source install/setup.bash`):

```bash
ros2 action send_goal /robotiq_2f_gripper_action robotiq_2f_gripper_msgs/action/MoveTwoFingerGripper "{target_position: 0.05, target_speed: 0.1, target_force: 0.1}"
```

You can specify a target postion in meters, i.e. distance between the gripper fingers, and a target speed and force in percent (between 0 to 1).

If you don't want to move the hardware, or don't have the hardware available, but still want to test the ROS2 action server you can launch the node with `fake_hardware:=true`:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py fake_hardware:=true
```

### robotiq_2f_gripper_interfaces

Adopted from [PickNikRobotics](https://github.com/PickNikRobotics/ros2_robotiq_gripper). The original license remains active.

This package should not be run or launched. Instead it provides the driver code for the gripper.

### robotiq_2f_gripper_msgs

This package should not be run or launched. Instead it provides a message template for the communication with the gripper.
