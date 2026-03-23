cmake_minimum_required(VERSION 3.8)
project(y1_moveit_ctrl)

# 设置 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED) # 常用依赖，以防万一

# 注册自定义服务文件 (ROS 2 方式)
# 注意：确保 srv/JointMoveitCtrl.srv 文件存在且路径正确
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JointMoveitCtrl.srv"
  DEPENDENCIES sensor_msgs
)

# 包含头文件目录
include_directories(
  include
)

# 创建可执行文件
add_executable(joint_moveit_ctrl_server moveit_server/joint_moveit_ctrl_server.cpp)

# 链接库
# ament_target_dependencies 会自动处理包含目录和链接库
ament_target_dependencies(joint_moveit_ctrl_server
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  std_srvs
)

# 链接生成的服务接口库
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(joint_moveit_ctrl_server "${cpp_typesupport_target}")

# 安装规则
install(TARGETS
  joint_moveit_ctrl_server
  DESTINATION lib/${PROJECT_NAME})

# 安装 srv 文件 (可选，为了其他包能依赖)
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  rosidl_default_runtime
)

# 必须添加这一行，否则 ros2 run 找不到节点
ament_package()