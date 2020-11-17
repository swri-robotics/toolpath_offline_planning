cmake_minimum_required(VERSION 2.8.3)
project(opp_database)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  opp_msgs
  opp_msgs_serialization
)

find_package(Qt5 REQUIRED COMPONENTS Sql)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_cpp
  CATKIN_DEPENDS
    roscpp
    roslib
    opp_msgs
    opp_msgs_serialization
  DEPENDS
    Boost
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_cpp
  src/${PROJECT_NAME}_interface_cpp.cpp
)
target_link_libraries(${PROJECT_NAME}_cpp
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  Qt5::Sql
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)

#  catkin_add_gtest(database_interface_cpp_test test/database_interface_cpp_test.cpp)
#  target_link_libraries(database_interface_cpp_test
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}_cpp
#    ${Boost_LIBRARIES}
#    Qt5::Sql
#  )
#endif()
