cmake_minimum_required(VERSION 2.8.3)
project(nexus_robot_2wd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  sensor_msgs
  tf2
  tf2_ros
  nexus_robot_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES nexus_robot_2wd
  # CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across pac
add_executable(nexus_robot src/nexus_robot_odom.cpp)
add_executable(nexus_robot_odometry src/nexus_odometry.cpp)
add_executable(nexus_robot_odometry_2 src/nexus_odometry_v2.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(nexus_robot
  ${catkin_LIBRARIES}
)

target_link_libraries(nexus_robot_odometry
  ${catkin_LIBRARIES}
)

target_link_libraries(nexus_robot_odometry_2
  ${catkin_LIBRARIES}
)
