cmake_minimum_required(VERSION 3.0.2)
project(riberry_startup)

message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
  OUTPUT_VARIABLE GCC_VERSION
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Configuring for ARM or AArch64 architecture.")
endif()

# To avoid issues with pub/sub in ROS topic on ARM architecture, compile with O2 optimization.
# https://github.com/ros/roscpp_core/pull/136
if(GCC_VERSION VERSION_LESS 9.4.0 AND (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") AND CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "-O2")
  message(STATUS "GCC version is less than 9.4.0 and target architecture is ARM. Setting CXX_FLAGS_RELEASE to -O2.")
endif()

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  Context.msg
  ImuFace.msg
  KeywordCandidates.msg
)

add_service_files(
  FILES
  RegisterContexts.srv
  SelectMotion.srv  
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

catkin_package(
  CATKIN_DEPENDS audio_common_msgs roscpp sensor_msgs std_msgs
)

add_executable(mpu6886_imu_publisher_node src/mpu6886_imu_publisher_node.cpp)
target_link_libraries(mpu6886_imu_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(i2c_audio_publisher src/i2c_audio_publisher.cpp)
target_link_libraries(i2c_audio_publisher
  ${catkin_LIBRARIES}
)

add_executable(i2c_button_state_publisher src/i2c_button_state_publisher.cpp)
target_link_libraries(i2c_button_state_publisher
  ${catkin_LIBRARIES}
)

add_executable(imu_filter src/imu_filter.cpp)
target_link_libraries(imu_filter
  ${catkin_LIBRARIES}
  m # include <math.h>
)
