#!/bin/sh

ros2 topic pub -1 /master_arm_right/mit_control y1_msg/msg/MitControlMode "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
kp: [0, 0, 0, 0, 0, 0, 0]
joint_position: [0, 0, 0, 0, 0, 0, 0]
kd: [0, 0, 0, 0, 0, 0, 0]
joint_velocity: [0, 0, 0, 0, 0, 0, 0]
torque: [-1, 0, 0, 0, 0, 0, 0]
" 