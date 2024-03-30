cmake_minimum_required(VERSION 3.0.2)
project(riberry_startup)

message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
  OUTPUT_VARIABLE GCC_VERSION
)

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
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
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
