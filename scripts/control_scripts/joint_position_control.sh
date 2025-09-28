#!/bin/sh

ros2 topic pub -1 /master_arm_right/joint_states y1_msg/msg/ArmJointPositionControl "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
joint_position: [0.6, -0.6, 0.6, 0.5, 0.4, 0]
joint_velocity: 5
gripper_stroke: 50
gripper_velocity: 5" 

# ros2 topic pub -1 /master_arm_left/joint_states y1_msg/msg/ArmJointPositionControl "header:
#   stamp: {sec: 0, nanosec: 0}
#   frame_id: ''
# joint_position: [0.6, -0.6, 0.6, 0.5, 0.4, 0]
# joint_velocity: 5
# gripper_stroke: 50
# gripper_velocity: 5" 