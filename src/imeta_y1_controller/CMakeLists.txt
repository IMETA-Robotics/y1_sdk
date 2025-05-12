cmake_minimum_required(VERSION 3.0.2)
project(imeta_y1_controller)

# set for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set for gdb debug
set(CMAKE_CXX_FLAGS "--std=c++17 -O0 -Og -ggdb")
add_compile_options(-std=c++17)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   urdf
#   kdl_parser
# )

# find_package(orocos_kdl REQUIRED)

# catkin_package()

# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${orocos_kdl_INCLUDE_DIRS}
# )

# add_executable(kdl_example src/kdl_example.cpp)
# target_link_libraries(kdl_example ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy 
  imeta_y1_msg
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/y1_sdk/lib/arm64/liby1_sdk_arm64.so")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
  set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/y1_sdk/lib/x64/liby1_sdk_x64.so")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}/lib/
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/y1_sdk)

# Creating an Import Library Target
add_library(y1_sdk_1 SHARED IMPORTED)
# Set the properties of the imported library
set_target_properties(y1_sdk_1 PROPERTIES
  IMPORTED_LOCATION ${LIB_PATH}
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lib/y1_sdk/"
)

add_executable(y1_controller
               y1_controller.cpp
               y1_controller_node.cpp)

target_link_libraries(y1_controller
                      PUBLIC ${catkin_LIBRARIES} y1_sdk_1 glog gflags)        

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# # install executables ans libraries
# install(TARGETS y1_controller
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# # install launch and config
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
