cmake_minimum_required(VERSION 2.8.3)
project(opp_startup)

find_package(catkin REQUIRED)

# Prevents certain runtime errors in Ubuntu 18.04
add_definitions(${PCL_DEFINITIONS})

catkin_package()

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
