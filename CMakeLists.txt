cmake_minimum_required(VERSION 3.0.2)
project(cona_tasho)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)

set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(PkgConfig REQUIRED)
find_package(CASADI REQUIRED)

set(JSONCPP_INCLUDE_DIRS "/usr/include/jsoncpp")
set(JSONCPP_LIBRARIES "jsoncpp")


# Declare catkin package and its dependencies
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_definitions(-DCASADI_RESOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/lib/casadi/")

# Declare executable targets and link libraries
add_executable(mpc_node src/mpc_node.cpp)
# target_include_directories(mpc_node ${CASADI_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(mpc_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES} ${CASADI_LIBRARIES})

add_executable(casadi2cpp src/casadi2cpp.cpp)
# target_include_directories(casadi2cpp ${CASADI_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(casadi2cpp ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES} ${CASADI_LIBRARIES})

add_executable(loadjson_example src/loadjson_example.cpp)
# target_include_directories(loadjson_example ${CASADI_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(loadjson_example ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES} ${CASADI_LIBRARIES})



# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)