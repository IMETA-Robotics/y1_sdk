#!/bin/bash

thread_num=$(($(nproc) - 1))

# y1 arm ros msgs
catkin_make install --pkg imeta_y1_msg -j${thread_num}

# y1 arm ros1 driver
catkin_make install --pkg imeta_y1_controller -j${thread_num}