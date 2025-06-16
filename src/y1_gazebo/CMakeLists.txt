cmake_minimum_required(VERSION 3.0.2)
project(y1_gazebo)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

find_package(catkin REQUIRED)

catkin_package()