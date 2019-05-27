cmake_minimum_required(VERSION 2.8.3)
project(e2box_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

set( sources
  src/main.cpp
  src/t_serial.cpp
  src/e2box_imu_9dofv4.cpp
)

set( headers
  src/t_serial.h
  src/e2box_imu_9dofv4.h
)

catkin_package(
  LIBRARIES e2box_imu
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${headers} ${sources})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)


