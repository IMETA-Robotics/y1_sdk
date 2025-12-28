#!/bin/sh

rostopic pub /master_arm_right/end_pose_control y1_msg/ArmEndPoseControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
end_pose: [[0.10077711241879538, 0.0011412381574326007, 0.30150638233312543, -0.007156568067465809, -0.0029598369049133127, 0.00785727781979032]]
joint_velocity: 3
gripper_stroke: 50
gripper_velocity: 3"
