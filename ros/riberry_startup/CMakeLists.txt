cmake_minimum_required(VERSION 2.8.3)
project(riberry_startup)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
)

# catkin_python_setup()

catkin_package(
  # CATKIN_DEPENDS message_runtime
  # sensor_msgs
  # DEPENDS
  # # INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  # LIBRARIES ${PROJECT_NAME}
)
