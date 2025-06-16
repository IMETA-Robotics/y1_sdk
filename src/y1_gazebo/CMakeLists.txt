cmake_minimum_required(VERSION 3.0.2)
project(y1_gazebo)

find_package(catkin REQUIRED COMPONENTS 
              gazebo_ros)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)