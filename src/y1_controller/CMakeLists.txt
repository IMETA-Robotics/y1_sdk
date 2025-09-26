cmake_minimum_required(VERSION 3.8)
project(y1_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(y1_msg REQUIRED)
find_package(glog REQUIRED)

# set for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set for gdb debug
set(CMAKE_CXX_FLAGS "--std=c++17 -O0 -Og -ggdb")
add_compile_options(-std=c++17)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/y1_sdk/lib/arm64/liby1_sdk_arm64.so")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
  set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/y1_sdk/lib/x64/liby1_sdk_x64.so")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/lib/
  ${PROJECT_SOURCE_DIR}
)

# Creating an Import Library Target
# add_library(y1_sdk SHARED IMPORTED)
# set_target_properties(y1_sdk PROPERTIES
#   IMPORTED_LOCATION ${LIB_PATH}
#   INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lib/y1_sdk/"
# )

add_executable(y1_controller
               y1_controller.cpp
               y1_controller_node.cpp)

target_include_directories(y1_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(y1_controller
  ${LIB_PATH}
  glog::glog
  rclcpp::rclcpp
)
ament_target_dependencies(y1_controller rclcpp std_msgs y1_msg ament_index_cpp)

install(TARGETS y1_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${LIB_PATH}
        DESTINATION lib/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING PATTERN "*.yaml")

install(DIRECTORY urdf/
        DESTINATION share/${PROJECT_NAME}/urdf
)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
