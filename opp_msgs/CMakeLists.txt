cmake_minimum_required(VERSION 2.8.3)
project(opp_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  noether_msgs
  sensor_msgs
  std_msgs
)

# Prevents certain runtime errors in Ubuntu 18.04
add_definitions(${PCL_DEFINITIONS})

add_message_files(
  DIRECTORY msg
  FILES
    ErrorType.msg
    Job.msg
    Part.msg
    ProcessType.msg
    ToolPath.msg
    ToolPathParams.msg
    TouchPoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES
      GetROISelection.srv
      GetPathSelectionMesh.srv
      GetPathSelectionCloud.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    noether_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    noether_msgs
    sensor_msgs
    std_msgs
)
