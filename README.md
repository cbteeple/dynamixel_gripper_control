# Dynamixel Gripper Control
A ROS package for controlling a dynamixel gripper as if it were a robotiq gripper.

## Dependencies
-  A Dynamixel-based gripper connected to an arduino running my [Dynamixel Servo firmware](https://github.com/cbteeple/dynamixel_servo).
- [ROS driver for Ctrl-P](https://github.com/cbteeple/pressure_control_cbt)

## Setup
1. Set up your hardware config file for serial communication per [Ctrl-P instructions](https://ctrl-p.cbteeple.com/latest/ros-driver/setup). You can ignore the part where it tells you to make a new ROS package. Just put files in the "**config**" folder in this package.
2. Set up your plotting config file using rqt_multiplot


## Usage
```bash
roslaunch dynamixel_gripper_control bringup_dynamixel.launch
```
