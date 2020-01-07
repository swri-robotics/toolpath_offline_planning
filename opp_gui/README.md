# opp_gui

This package provides GUI implementations for various applications

## Tool Path Planner

The tool path planner GUI is an RViz panel designed to facilitate the setup of new models and tool path
planning on those models. The GUI also hosts an interface to the database for saving this information

### Usage

1. Build and source the package
1. Run the application
    ```
    roslaunch opp_startup planner_application.launch
    ```
1. Follow the order of operations of the GUI

## Widget Demos

Each stand-alone widget has a node for demonstrating its functionality

### Touch point editor
```
rosrun opp_gui touch_point_editor_demo_app
rosrun rviz rviz
```

To add a point using the mouse follow these steps:

  1. Add the `Circle Tool Cursor` tool to RViz (Should already be included.)
  2. Click the `Select with Mouse` button in the widget
  3. Activate the `Circle Tool Cursor` tool
  4. Select a location on a mesh in the Rviz render panel

### Tool path editor

```
rosrun noether surface_raster_planner_server
rosrun opp_gui tool_path_editor_demo_app
rosrun rviz rviz
```

### Tool path planner
```
rosrun noether surface_raster_planner_server
rosrun opp_gui tool_path_planner_demo_app
rosrun rviz rviz
```

### Database Browser Widget
```
roslaunch opp_startup opp.launch
rosrun opp_gui database_browser_demo_app
```
