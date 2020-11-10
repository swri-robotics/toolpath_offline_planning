# Toolpath Offline Planner

[![Build Status](https://travis-ci.com/swri-robotics/Toolpath-Offline-Planning.svg?branch=master)](https://travis-ci.com/swri-robotics/Toolpath-Offline-Planning)
[![Github Issues](https://img.shields.io/github/issues/swri-robotics/Toolpath-Offline-Planning.svg)](http://github.com/swri-robotics/Toolpath-Offline-Planning/issues)

ROS packages for generating offline toolpaths from CAD.  Built to run on Ubuntu 16.04 or 18.04.

## TODO
1) More thorough usage instructions
2) Pretty pictures

## Install instructions
1) Clone the repo into your workspace. Then from the workspace root run `wstool init src dependencies.rosinstall`

2) Run rosdep from the root of your workspace to pull dependencies `rosdep install --from-paths src --ignore-src -r -y`

    - Ensure that the following dependencies were downloaded - they are necessary to use the C++ database interface

    ```
    libqt5sql5-mysql
    libmysqlclient-dev
    libssl-dev
    ```

3) Follow the instructions at the [noether repository](https://github.com/ros-industrial/noether) for installing the correct versions of PCL (1.9+) and VTK (1.7+).

    - Noether requires at least PCL 1.9 and VTK 7.1


4) Build the workspace `catkin build`

5) Source the workspace

6) Set up the MySQL database.
    - an "offline_generated_paths" folder needs to be created. (Press CTRL+H to show hidden folders): ~/.local/share/offline_generated_paths
    - install MySQL and make sure you have root/admin access.
        a) If you do not already have mysql installed:
            1. Install **mysql**:

                Follow the directions in the [mysql website to install the debian](https://dev.mysql.com/doc/mysql-apt-repo-quick-guide/en/#apt-repo-fresh-install)

                *NOTE: If you set a password you will need to remember it in order to log in into a mysql session.
            2. Initialize **mysql**:
                Run the following command as root or with sudo in order to initialize mysql.  We'll use the `--initialize-insecure` option at first:
                ```
                mysqld --initialize-insecure --user=mysql
                ```

                Remember to use `sudo` if you aren't logged in as **root**. For more details go to the [mysql webpage](https://dev.mysql.com/doc/refman/8.0/en/data-directory-initialization-mysqld.html)
            3. Set Password
                - Connect to the server as root with no password
                    ```
                    mysql -u root --skip-password
                    ```
                    **NOTE: Should this failed because a password is required then log in as follows:**
                    ```
                    mysql -u root -p
                    ```
                    *Enter your password when asked.*
                - You'll now be in a mysql console session, next set `0000` as the password for the root account if one hasn't been set yet.  (You should probably use something more secure than '0000', but what would be the fun in that?)
                    ```
                    ALTER USER 'root'@'localhost' IDENTIFIED BY '0000';
                    ```
                - Press Ctrl-D to log out
                - Log in again using the password
                    ```
                    mysql -u root -p
                    ```

                *NOTES:*
                - *For more info see the [mysql webpage](https://dev.mysql.com/doc/refman/8.0/en/default-privileges.html)*
                - *Additional help on resetting the password can be found [here](https://www.digitalocean.com/community/tutorials/how-to-import-and-export-databases-and-reset-a-root-password-in-mysql)*

        b) If you do already have mysql installed, but you do not know the root password:
            - My favorite hack for getting into a mysql server with an unknown password is detailed [here](https://askubuntu.com/questions/766900/mysql-doesnt-ask-for-root-password-when-installing), in the answer by pwxcoo.

    - an "opp" database needs to be created manually.  The necessary tables will be automatically generated.
        ```
        $ mysql -u root -p
        mysql> CREATE DATABASE opp;
        mysql> SHOW DATABASES;
        ```
    - 'ros-client' mysql user needs to be created and its password should be '0000'.  It only needs permission to the opp database created above.  If you want it to have all permissions, you can use the command below.
        ```
        $ mysql -u root -p
        mysql> CREATE USER 'ros-client'@'localhost' IDENTIFIED BY '0000';
        mysql> GRANT ALL PRIVILEGES ON opp.* TO 'ros-client'@'localhost' WITH GRANT OPTION;
        ```
        **NOTE: This example uses '0000' for the password. You could use a safer password, but this one is currently hardcoded.**
        See [here](https://dev.mysql.com/doc/refman/8.0/en/adding-users.html) for more info on user privileges.


## Offline Tool Path Planner

The tool path planner GUI is an RViz panel designed to facilitate the setup of new models and tool path
planning on those models. The GUI also hosts an interface to the database for saving this information.

### Usage

1. Build and source the package
2. Run the application
    ```
    roslaunch opp_startup planner_application.launch
    ```
3. Follow the order of operations of the GUI

