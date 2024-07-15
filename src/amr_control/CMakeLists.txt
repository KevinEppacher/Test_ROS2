cmake_minimum_required(VERSION 3.5)
project(amr_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/publisher_node.py
  scripts/subscriber_node.py
  scripts/casadi_example.py
  scripts/optimal_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, maps, and configuration files
install(DIRECTORY
    launch
    map
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
