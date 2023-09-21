cmake_minimum_required(VERSION 3.0.2)
project(cona_tasho)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES md_driver
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/md_driver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

