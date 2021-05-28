cmake_minimum_required(VERSION 2.8.3)
project(opp_gui)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  eigen_conversions
  eigen_stl_containers
  geometric_shapes
  geometry_msgs
  noether_msgs
  heat_msgs
  pluginlib
  roscpp
  opp_area_selection
  opp_path_selection
  opp_msgs
  opp_database
  pcl_ros
  rviz
  shape_msgs
  std_srvs
  visualization_msgs
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Sql
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_panels
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_widgets
  CATKIN_DEPENDS
    actionlib
    eigen_conversions
    eigen_stl_containers
    geometric_shapes
    geometry_msgs
    noether_msgs
    heat_msgs
    pluginlib
    roscpp
    opp_area_selection
    opp_path_selection
    opp_msgs
    opp_database
    pcl_ros
    rviz
    shape_msgs
    std_srvs
    visualization_msgs
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(opp_gui_UIS
  ui/list_editor.ui
  ui/polygon_area_selection_widget.ui
  ui/polyline_path_selection_widget.ui
  ui/segmentation_parameters_editor.ui
  ui/surface_selection_combo_widget.ui
  ui/tool_path_parameters_editor.ui
  ui/tool_path_planner.ui
  ui/touch_point_parameters_editor.ui
)

set(opp_gui_widget_HEADERS
  include/${PROJECT_NAME}/widgets/list_editor_widget_base.h
  include/${PROJECT_NAME}/widgets/polygon_area_selection_widget.h
  include/${PROJECT_NAME}/widgets/polyline_path_selection_widget.h
  include/${PROJECT_NAME}/widgets/segmentation_parameters_editor_widget.h
  include/${PROJECT_NAME}/widgets/surface_selection_combo_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_editor_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_parameters_editor_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_planner_widget.h
  include/${PROJECT_NAME}/widgets/touch_point_editor_widget.h
  include/${PROJECT_NAME}/widgets/touch_point_parameters_editor_widget.h
)

set(opp_gui_widget_SOURCES
  src/widgets/polygon_area_selection_widget.cpp
  src/widgets/polyline_path_selection_widget.cpp
  src/widgets/segmentation_parameters_editor_widget.cpp
  src/widgets/surface_selection_combo_widget.cpp
  src/widgets/tool_path_editor_widget.cpp
  src/widgets/tool_path_parameters_editor_widget.cpp
  src/widgets/tool_path_planner_widget.cpp
  src/widgets/touch_point_editor_widget.cpp
  src/widgets/touch_point_parameters_editor_widget.cpp
)

set(opp_gui_panel_HEADERS
  include/${PROJECT_NAME}/tool_path_planner_panel.h
)

set(opp_gui_panel_SOURCES
  src/tool_path_planner_panel.cpp
)

# Run the MOC on the Qt-related UIs and headers
qt5_wrap_ui(opp_gui_UIS_H
  ${opp_gui_UIS}
)
qt5_wrap_cpp(opp_gui_widget_MOCS
  ${opp_gui_widget_HEADERS}
)
qt5_wrap_cpp(opp_gui_panel_MOCS
  ${opp_gui_panel_HEADERS}
)

# Utilities library
add_library(${PROJECT_NAME}_utils
  src/utils.cpp
)
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Qt5::Core
)

# Qt Widgets
add_library(${PROJECT_NAME}_widgets
  ${opp_gui_UIS_H}
  ${opp_gui_widget_MOCS}
  ${opp_gui_widget_SOURCES}
)
target_link_libraries(${PROJECT_NAME}_widgets
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_utils
  Qt5::Widgets
  Qt5::Sql
)

# Rviz Panel
add_library(${PROJECT_NAME}_panels
  ${opp_gui_panel_MOCS}
  ${opp_gui_panel_SOURCES}
)
target_link_libraries(${PROJECT_NAME}_panels
  ${PROJECT_NAME}_widgets
)

######################
## Demo Executables ##
######################

# Segmentation
add_executable(segmentation_demo_app
  src/nodes/segmentation_demo_app.cpp
)
target_link_libraries(segmentation_demo_app
  ${PROJECT_NAME}_widgets
)

# Tool Path Editor
add_executable(tool_path_editor_demo_app
  src/nodes/tool_path_editor_demo_app.cpp
)
target_link_libraries(tool_path_editor_demo_app
  ${PROJECT_NAME}_widgets
)

# Tool Path Planner
add_executable(tool_path_planner_demo_app
  src/nodes/tool_path_planner_demo_app.cpp
)
target_link_libraries(tool_path_planner_demo_app
  ${PROJECT_NAME}_widgets
)

# Touch Point Editor
add_executable(touch_point_editor_demo_app
  src/nodes/touch_point_editor_demo_app.cpp
)
target_link_libraries(touch_point_editor_demo_app
  ${PROJECT_NAME}_widgets
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_panels
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_widgets
    segmentation_demo_app
    tool_path_editor_demo_app
    tool_path_planner_demo_app
    touch_point_editor_demo_app
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui
)

install(
  FILES
    plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
