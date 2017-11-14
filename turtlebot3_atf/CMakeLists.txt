cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_atf)

find_package(catkin REQUIRED COMPONENTS
  atf_core
  roslaunch
  rospy
  turtlebot3
  turtlebot3_simulations
)
catkin_package(
   CATKIN_DEPENDS rospy
)

###########
## Build ##
###########

#############
## Install ##
#############

install(PROGRAMS scripts/application.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############
atf_test()
