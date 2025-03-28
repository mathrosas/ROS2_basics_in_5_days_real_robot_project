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
find_package(rosidl_default_generators REQUIRED)

# Define your service file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindWall.srv"
)

# Include directories
include_directories(include)

# Add executables
add_executable(wall_follower_node src/wall_follower.cpp)
ament_target_dependencies(wall_follower_node rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(wall_finder_node src/wall_finder.cpp)
ament_target_dependencies(wall_finder_node rclcpp sensor_msgs geometry_msgs std_msgs)

# Link the rosidl interfaces
rosidl_target_interfaces(wall_follower_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(wall_finder_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install nodes
install(TARGETS wall_follower_node wall_finder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (optional, if you have launch files)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
