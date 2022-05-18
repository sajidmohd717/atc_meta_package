cmake_minimum_required(VERSION 2.8.3)
project(atc_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   BoundingBox.msg
   AprilTag_Has_Solution.msg
   Detector.msg
   Detector_Has_Solution.msg
   AGVStatus.msg
   Stop_To_STM.msg
   Latch.msg
   waypoint_msg.msg
   waypoint_group.msg
   waypointArray.msg
   RunSpecificWp.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Update_Movement_Mode.srv
   Navigate_To_Trolley.srv
   Dock_To_Tag.srv
   #AprilTag_Has_Solution.srv
   Click_To_Turn.srv
   Save_Wp.srv
   Load_Wp.srv
   Run_Wp.srv
   Run_Specific_Wp.srv
   Stop_Wp.srv
   Delete_Wp.srv
   Groups_Wp.srv
   Wp_2_Group.srv
   #*New additions (28th July 2021)
   Reset_ClearCostMap.srv
   #Goto_Home.srv
   #Goto_Trolley_DropOff.srv
   #Charge_AGV.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   move_base_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


## Install rules
install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Note: atc_waypoint waypoint files are now moved to atc_msgs
install(DIRECTORY files 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
  

