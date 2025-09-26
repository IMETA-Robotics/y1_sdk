#!/bin/bash

# y1 arm urdf description
# catkin_make install --pkg y1_description -j${thread_num}

# y1 arm gazebo
# catkin_make install --pkg y1_gazebo -j${thread_num}

# y1 arm ros msgs
colcon build --packages-select y1_msg 
# y1 arm ros2 driver
colcon build --packages-select y1_controller 