include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})

set(Y1_SDK_SRCS
    traffic_decider.cpp)

add_library(y1_sdk SHARED ${Y1_SDK_SRCS})
target_link_libraries(y1_sdk PUBLIC
    ${catkin_LIBRARIES}
)

# 根据架构设置安装目录和库名称
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(ARCH_DIR "lib/arm64")
    set(LIB_NAME "y1_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(ARCH_DIR "lib/x64")
    set(LIB_NAME "y1_sdk_x64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set_target_properties(y1_sdk PROPERTIES OUTPUT_NAME LIB_NAME)

# 安装库文件到对应的架构目录，并重命名库文件
# install(TARGETS y1_sdk
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${ARCH_DIR}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/${ARCH_DIR}
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${ARCH_DIR}
# )

# 设置安装前缀为项目源目录
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}")
install(TARGETS y1_sdk
    LIBRARY DESTINATION ${ARCH_DIR}
)

# install include directories
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE)
