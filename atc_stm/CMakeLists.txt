cmake_minimum_required(VERSION 2.8.3)
project(atc_stm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  atc_msgs
  atc_utils
  apriltag_ros
#  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Update_Movement_Mode.srv
#   PE_Has_Solution.srv
#   AprilTag_Has_Solution.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   atc_msgs
# )


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
#  INCLUDE_DIRS include
#  LIBRARIES atc_utils
#  CATKIN_DEPENDS roscpp std_msgs atc_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(atc_stm 
src/atc_stm_main.cpp 
src/stm_utils.cpp 
src/callbacks.cpp 
src/StateTransitionManager.cpp 
src/AprilTagLogic/AprilTagLogic.cpp 
src/AprilTagLogic/apriltag_utils.cpp
src/ManualModeLogic/ManualModeLogic.cpp
src/DetectorLogic/DetectorLogic.cpp 
src/WaypointLogic/WaypointLogic.cpp)

target_link_libraries(atc_stm ${catkin_LIBRARIES})


## Install rules
install(TARGETS atc_stm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Note: atc_waypoint waypoint files are now moved to atc_msgs
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(FILES
#  plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

#install(DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#catkin_install_python(PROGRAMS scripts/send_test_msgs.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



