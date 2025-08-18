#!/bin/sh

rostopic pub /y1/arm_joint_position_control imeta_y1_msg/ArmJointPositionControl "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
arm_joint_position: [0, -2.5, 2.8, -0.2, 0, 0]
arm_joint_velocity: 0.8
gripper: 50"
