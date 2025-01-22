cmake_minimum_required(VERSION 3.8)
project(my_teleop_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)

# Link dependencies to the executable
ament_target_dependencies(teleop_twist_keyboard rclcpp geometry_msgs)

# Install the executable
install(TARGETS
  teleop_twist_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
