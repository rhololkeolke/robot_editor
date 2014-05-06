cmake_minimum_required(VERSION 2.8.3)
project(robot_editor)

find_package(catkin REQUIRED COMPONENTS
  rviz
  urdf
)

# catkin_python_setup()

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#############
## Testing ##
#############