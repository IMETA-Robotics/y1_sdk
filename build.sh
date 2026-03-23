#!/bin/bash

# y1 arm ros msgs
colcon build --packages-select y1_msg 
# y1 description
colcon build --packages-select y1_description
# y1 arm ros2 driver
colcon build --packages-select y1_controller
# y1 arm no gripper ros2 moveit 
colcon build --packages-select y1_no_gripper_moveit
# y1 arm with gripper ros2 moveit
colcon build --packages-select y1_with_gripper_moveit
# y1 arm y1_moveit_ctrl
colcon build --packages-select y1_moveit_ctrl
# y1 arm y1_gazebo
colcon build --packages-select y1_gazebo