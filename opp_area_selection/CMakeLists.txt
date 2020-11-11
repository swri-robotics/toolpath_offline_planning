cmake_minimum_required(VERSION 2.8.3)
project(opp_area_selection)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  opp_msgs_serialization
  opp_msgs
  pcl_ros
  sensor_msgs
  std_srvs
  tf
  tf_conversions
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    opp_msgs_serialization
    opp_msgs
    pcl_ros
    sensor_msgs
    std_srvs
    tf
    tf_conversions
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Library containing tools for specifying regions
# and determining inlying points
add_library(${PROJECT_NAME}
  src/area_selector.cpp
  src/selection_artist.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Node that runs area selection service
add_executable(${PROJECT_NAME}_node
  src/area_selection_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# Install built targets
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
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
