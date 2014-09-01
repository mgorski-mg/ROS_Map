cmake_minimum_required(VERSION 2.8.3)
project(differential_drive)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf rospy)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime rosconsole rospy geometry_msgs
#  DEPENDS system_lib
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/twist_to_motors
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)