cmake_minimum_required(VERSION 2.8.3)
project(vrpn_client_ros)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp roslaunch roslint tf2_ros)
find_package(VRPN REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vrpn_client_ros
  CATKIN_DEPENDS geometry_msgs tf2_ros
  DEPENDS VRPN
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror -Wno-unknown-pragmas")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VRPN_INCLUDE_DIR}
)

add_library(vrpn_client_ros
  src/vrpn_client_ros.cpp
)
target_link_libraries(vrpn_client_ros
  ${catkin_LIBRARIES}
  ${VRPN_LIBRARIES}
)

add_executable(vrpn_tracker_node
  src/vrpn_tracker_node.cpp)
add_dependencies(vrpn_tracker_node vrpn_client_ros)
target_link_libraries(vrpn_tracker_node
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

add_executable(vrpn_client_node
  src/vrpn_client_node.cpp)
add_dependencies(vrpn_client_node vrpn_client_ros)
target_link_libraries(vrpn_client_node
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

file(GLOB_RECURSE SOURCE_FILES src *.cpp)
file(GLOB_RECURSE HEADER_FILES include *.h)

roslint_cpp(
  ${SOURCE_FILES}
  ${HEADER_FILES}
  )

install(TARGETS vrpn_client_ros vrpn_tracker_node vrpn_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
