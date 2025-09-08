cmake_minimum_required(VERSION 3.0.2)
project(y1_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

# 包含头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_states_ctrl joint_states_ctrl/joint_states_ctrl.cpp)
target_link_libraries(joint_states_ctrl ${catkin_LIBRARIES})
# add_dependencies(joint_states_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})