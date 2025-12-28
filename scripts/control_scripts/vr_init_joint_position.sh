#!/bin/sh

rostopic pub /master_arm_right/joint_states y1_msg/ArmJointPositionControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_position: [0.015, -0.622, 0.372, 0.267, 0.011, 0.0]
joint_velocity: 3
gripper_stroke: 50
gripper_velocity: 3" 