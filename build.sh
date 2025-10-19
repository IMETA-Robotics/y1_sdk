#!/bin/bash

# y1 arm ros msgs
colcon build --packages-select y1_msg 
# y1 description
colcon build --packages-select y1_description
# y1 arm ros2 driver
colcon build --packages-select y1_controller