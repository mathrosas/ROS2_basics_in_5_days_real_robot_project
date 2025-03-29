cmake_minimum_required(VERSION 3.8)
project(wall_follower)

# Enable C++17 explicitly
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Define your service file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindWall.srv"
  "action/OdomRecord.action"
  DEPENDENCIES geometry_msgs
)

# Add executables
add_executable(wall_follower_node src/wall_follower.cpp)
ament_target_dependencies(wall_follower_node rclcpp sensor_msgs geometry_msgs std_msgs rclcpp_action)

add_executable(wall_finder_node src/wall_finder.cpp)
ament_target_dependencies(wall_finder_node rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(record_odom_node src/record_odom.cpp)
ament_target_dependencies(record_odom_node rclcpp sensor_msgs geometry_msgs std_msgs rclcpp_action nav_msgs)

# Link the rosidl interfaces
rosidl_target_interfaces(wall_follower_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(wall_finder_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(record_odom_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install nodes
install(TARGETS
  wall_follower_node
  wall_finder_node
  record_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (optional, if you have launch files)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
