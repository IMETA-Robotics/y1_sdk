#!/bin/sh

# rostopic pub /y1_controller/arm_joint_position_control imeta_y1_msg/ArmJointPositionControl "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# arm_joint_position: [0.9, -0.7, 0.8, 0.5, 0.4, -0.4]
# arm_joint_velocity: [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
# gripper: 0.0"

rostopic pub /y1_controller/arm_joint_position_control imeta_y1_msg/ArmJointPositionControl "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
arm_joint_position: [0, 0, 0, 0, 0, 0]
arm_joint_velocity: [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
gripper: 0.0"
