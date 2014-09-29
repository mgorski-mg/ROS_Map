cmake_minimum_required(VERSION 2.8.3)
project(trobot)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp tf)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Odometry.msg
  MotorAmps.msg
  Encoder.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES roboteQLib
  CATKIN_DEPENDS message_runtime rosconsole
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(roboteQLib src/RoboteQNode/RoboteqDevice.cpp)
target_link_libraries(roboteQLib ${rosconsole_LIBRARIES})

add_executable(consoleTeleop src/ConsoleTeleop/consoleTeleop.cpp)
target_link_libraries(consoleTeleop ${catkin_LIBRARIES})
add_dependencies(consoleTeleop trobot_gencpp)

add_executable(RoboteQNode src/RoboteQNode/RoboteQNode.cpp)
target_link_libraries(RoboteQNode roboteQLib ${catkin_LIBRARIES})
add_dependencies(RoboteQNode trobot_gencpp)

add_executable(OdometryNode src/OdometryNode/OdometryNode.cpp)
target_link_libraries(OdometryNode ${catkin_LIBRARIES})
add_dependencies(OdometryNode trobot_gencpp)

add_executable(AutonomicNode src/AutonomicNode/AutonomicNode.cpp)
target_link_libraries(AutonomicNode ${catkin_LIBRARIES})
add_dependencies(AutonomicNode trobot_gencpp)