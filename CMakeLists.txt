cmake_minimum_required(VERSION 2.8.3)
project(vrpn_ros)

list(APPEND	CMAKE_MODULE_PATH	${vrpn_ros_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED roscpp tf2_ros)

find_package(VRPN REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vrpn_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${VRPN_INCLUDE_DIR})

## Declare a cpp library
# add_library(vrpn_ros
#   src/${PROJECT_NAME}/vrpn_ros.cpp
# )

## Declare a cpp executable
 add_executable(vrpn_ros_node src/vrpn_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vrpn_ros_node vrpn_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(vrpn_ros_node
   ${catkin_LIBRARIES}
   ${VRPN_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vrpn_ros vrpn_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vrpn_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
