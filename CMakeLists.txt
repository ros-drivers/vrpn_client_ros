cmake_minimum_required(VERSION 2.8.3)
project(vrpn_ros)

list(APPEND	CMAKE_MODULE_PATH	${vrpn_ros_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED geometry_msgs roscpp roslaunch roslint tf2_ros)
find_package(VRPN REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vrpn_ros
  CATKIN_DEPENDS geometry_msgs tf2_ros
  DEPENDS VRPN
)

include_directories(include ${catkin_INCLUDE_DIRS} ${VRPN_INCLUDE_DIR})

add_library(vrpn_ros
  src/vrpn_ros.cpp
)
target_link_libraries(vrpn_ros
  ${catkin_LIBRARIES}
  ${VRPN_LIBRARIES}
)

add_executable(vrpn_tracker_node
  src/vrpn_tracker_node.cpp)
add_dependencies(vrpn_tracker_node vrpn_ros)
target_link_libraries(vrpn_tracker_node
  vrpn_ros
  ${catkin_LIBRARIES}
)

add_executable(vrpn_client_node
  src/vrpn_client_node.cpp)
add_dependencies(vrpn_client_node vrpn_ros)
target_link_libraries(vrpn_client_node
  vrpn_ros
  ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

file(GLOB_RECURSE SOURCE_FILES src *.cpp)
file(GLOB_RECURSE HEADER_FILES include *.h)

roslint_cpp(
  ${SOURCE_FILES}
  ${HEADER_FILES}
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
