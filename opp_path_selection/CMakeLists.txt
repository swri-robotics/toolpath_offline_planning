cmake_minimum_required(VERSION 2.8.3)
project(opp_path_selection)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  shape_msgs
  opp_msgs_serialization
  roscpp
  roslib
  opp_msgs
  sensor_msgs
  std_srvs
  tf
  tf_conversions
  visualization_msgs
  pcl_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    path_selection
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    shape_msgs
    opp_msgs_serialization
    roscpp
    roslib
    opp_msgs
    sensor_msgs
    std_srvs
    tf
    tf_conversions
    visualization_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library containing tools for specifying regions
# and determining inlying points
add_library(path_selection
  src/path_selector.cpp
  src/path_selection_artist.cpp
)

target_link_libraries(path_selection
  ${catkin_LIBRARIES}
)

add_dependencies(path_selection
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Node that runs area selection service
add_executable(path_selection_node
  src/path_selection_node.cpp
)

target_link_libraries(path_selection_node
  path_selection
  ${catkin_LIBRARIES}
)

add_dependencies(path_selection_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Install built targets
install(TARGETS
    path_selection
    path_selection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES config/path_selection_parameters.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
