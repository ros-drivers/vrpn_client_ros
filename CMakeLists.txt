
cmake_minimum_required(VERSION 3.5)
project(vrpn_client_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(VRPN REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(include)

add_library(vrpn_client_ros
            src/vrpn_client_ros.cpp)
ament_target_dependencies(vrpn_client_ros rclcpp std_msgs geometry_msgs tf2 tf2_ros VRPN)

add_executable(vrpn_tracker_node src/vrpn_tracker_node.cpp)
target_link_libraries(vrpn_tracker_node vrpn_client_ros)
ament_target_dependencies(vrpn_tracker_node rclcpp std_msgs geometry_msgs tf2 tf2_ros VRPN)

add_executable(vrpn_client_node src/vrpn_client_node.cpp)
target_link_libraries(vrpn_client_node vrpn_client_ros)
ament_target_dependencies(vrpn_client_node rclcpp std_msgs geometry_msgs tf2 tf2_ros VRPN)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  vrpn_client_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  vrpn_tracker_node
  vrpn_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()