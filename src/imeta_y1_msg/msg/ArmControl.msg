std_msgs/Header header

# x y z roll pitch yaw
float64[6] arm_end_pose

# 6 joint position
float64[6] arm_joint_position

# individual gripper control
float64 gripper

int32 control_mode