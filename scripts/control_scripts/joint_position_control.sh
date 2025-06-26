#!/bin/sh

rostopic pub /y1_controller/arm_joint_position_control imeta_y1_msg/ArmJointPositionControl "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
arm_joint_position: [0.6, 0, 0, 0, 0, 0]
gripper: 0.0"
