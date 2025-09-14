#!/bin/sh

rostopic pub /y1/arm_joint_position_control y1_msg/ArmJointPositionControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_position: [0, -0, 0, 0, 0, 0]
joint_velocity: 5
gripper_stroke: 0 
gripper_velocity: 5" 
