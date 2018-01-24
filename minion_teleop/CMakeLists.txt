cmake_minimum_required(VERSION 2.8.3)
project(minion_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy roscpp geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy roscpp geometry_msgs joy
  DEPENDS
)

###########
## Build ##
###########

add_executable(minion_teleop_joy src/minion_joy.cpp)
target_link_libraries(minion_teleop_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/minion_teleop_key scripts/cmd2vec3
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS minion_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
