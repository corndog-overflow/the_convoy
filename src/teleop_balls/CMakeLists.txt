cmake_minimum_required(VERSION 3.5)
project(teleop_balls)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build the executable
add_executable(teleop_balls src/teleop_balls.cpp)

ament_target_dependencies(teleop_balls
  rclcpp
  std_msgs
  geometry_msgs
)

# Install the binary
install(TARGETS
  teleop_balls
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

