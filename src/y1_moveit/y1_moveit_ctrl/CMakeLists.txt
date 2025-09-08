cmake_minimum_required(VERSION 3.0.2)
project(y1_moveit_ctrl)

## Find catkin macros and libraries
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  roscpp
  moveit_ros_planning_interface
  sensor_msgs
  message_generation
)

# 添加服务文件
add_service_files(
  FILES
  JointMoveitCtrl.srv  # 这里是你的自定义服务文件
)


# 生成消息和服务文件
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

# 包含头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_moveit_ctrl_server moveit_server/joint_moveit_ctrl_server.cpp)
target_link_libraries(joint_moveit_ctrl_server ${catkin_LIBRARIES})

add_executable(test_joint_moveit_ctrl moveit_server/joint_moveit_ctrl.cpp)
target_link_libraries(test_joint_moveit_ctrl ${catkin_LIBRARIES})

# # 安装 Python 脚本
# catkin_install_python(PROGRAMS
#   scripts/joint_moveit_ctrl_server.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
