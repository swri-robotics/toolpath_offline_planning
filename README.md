# toolpath_offline_planning
ROS-based GUI interface for generating tool paths from mesh models offline

## Build Status
Platform | CI Status
---------|:---------
Linux (Xenial) | [![Xenial-Build](https://github.com/swri-robotics/toolpath_offline_planning/workflows/Xenial-Build/badge.svg)](https://github.com/swri-robotics/toolpath_offline_planning/actions)
Linux (Bionic) | [![Bionic-Build](https://github.com/swri-robotics/toolpath_offline_planning/workflows/Bionic-Build/badge.svg)](https://github.com/swri-robotics/toolpath_offline_planning/actions)
Linux (Focal) | [![Focal-Build](https://github.com/swri-robotics/toolpath_offline_planning/workflows/Focal-Build/badge.svg)](https://github.com/swri-robotics/toolpath_offline_planning/actions)

[![Clang-Formatting](https://github.com/swri-robotics/toolpath_offline_planning/workflows/Clang-Formatting/badge.svg)](https://github.com/swri-robotics/toolpath_offline_planning/actions)

[![Github Issues](https://img.shields.io/github/issues/swri-robotics/Toolpath-Offline-Planning.svg)](http://github.com/swri-robotics/Toolpath-Offline-Planning/issues)


## Dependencies
### Noether
This repository depends on the [Noether](https://github.com/ros-industrial/noether) tool path planning repository which utilizes custom versions of PCL and VTK.
Follow its instructions for installing the required dependencies

### SQL Database (Optional)
The following debian packages are required to use the database functionality. Please ensure they have been installed before building this repository.
- `libqt5sql5-mysql`
- `libmysqlclient-dev`
- `libssl-dev`

## Install instructions
1. After cloning the repository, install its dependencies into your workspace.
    ```bash
    cd <path_to_workspace>
    wstool init src src/toolpath_offline_planning/dependencies.rosinstall
    rosdep install --from-paths src --ignore-src -r -y
    ```

1. Build and source the workspace
    ```bash
    # Optionally skip building the SQL database interface package
    catkin config -a --blacklist opp_database
    # Build the repository
    catkin build
    source devel/setup.bash
    ```

## Offline Tool Path Planner GUI

The tool path planner GUI is an RViz panel designed to facilitate the setup of new models and tool path
planning on those models. The GUI also hosts an interface to the database for saving this information.

### Usage

1. Build and source the package
1. Run the application
    ```
    roslaunch opp_startup planner_application.launch
    ```
1. Follow the order of operations of the GUI
   Load an object
   define touchoff and verification points
   save the object to the database
   create a new job
   run one of the many tool path planners
   save the job to a database
