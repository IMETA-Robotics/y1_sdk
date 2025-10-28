#!/bin/bash

thread_num=$(($(nproc) - 1))

# y1 arm urdf description
catkin_make install --only-pkg-with-deps y1_description -j${thread_num}

# y1 arm gazebo
# catkin_make install --only-pkg-with-deps y1_gazebo -j${thread_num}

# y1 arm mujoco
# catkin_make install --only-pkg-with-deps y1_mujoco -j${thread_num}

# y1 arm y1_moveit_ctrl
# catkin_make install --only-pkg-with-deps y1_moveit_ctrl -j${thread_num}

# y1 arm y1_no_gripper_moveit
# catkin_make install --only-pkg-with-deps y1_no_gripper_moveit -j${thread_num}

# y1 arm y1_with_gripper_moveit
# catkin_make install --only-pkg-with-deps y1_with_gripper_moveit -j${thread_num}

# y1 arm ros msgs
catkin_make install --only-pkg-with-deps y1_msg -j${thread_num}

# y1 arm ros1 driver
catkin_make install --only-pkg-with-deps y1_controller -j${thread_num}