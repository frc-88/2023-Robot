##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(tj2_interfaces)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  rosjava_build_tools
  std_msgs
  geometry_msgs
  message_generation
)
catkin_rosjava_setup(publish)

add_message_files(
  FILES
  ZoneInfo.msg
  Labels.msg
  ZoneInfoArray.msg
  Waypoint.msg
  MatchPeriod.msg
  ZoneArray.msg
  GameObject.msg
  WaypointArray.msg
  SwerveModule.msg
  UVBoundingBox.msg
  XYZBoundingBox.msg
  SwerveMotor.msg
  Zone.msg
  XYZKeypoint.msg
  UVKeypoint.msg
  GameObjectsStamped.msg
  NoGoZones.msg
  Match.msg
  NavX.msg
)

add_service_files(
  FILES
  OdomReset.srv
  SaveTF.srv
  GetWaypoints.srv
  GetWaypoint.srv
  GetAllWaypoints.srv
  DeleteWaypoint.srv
  SaveWaypoint.srv
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
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package()

##############################################################################
# Installation
##############################################################################

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/tj2_interfaces/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/tj2_interfaces)
