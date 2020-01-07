cmake_minimum_required(VERSION 2.8.3)
project(opp_area_selection)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  opp_msgs_serialization
  roscpp
  roslib
  opp_msgs
  sensor_msgs
  std_srvs
  tf
  tf_conversions
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.9 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    area_selection
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
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
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Library containing tools for specifying regions
# and determining inlying points
add_library(area_selection
  src/area_selector.cpp
  src/selection_artist.cpp
)
target_link_libraries(area_selection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(area_selection
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Node that runs area selection service
add_executable(area_selection_node
  src/area_selection_node.cpp
)
target_link_libraries(area_selection_node
  area_selection
  ${catkin_LIBRARIES}
)
add_dependencies(area_selection_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Install built targets
install(TARGETS
    area_selection
    area_selection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES config/area_selection_parameters.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
