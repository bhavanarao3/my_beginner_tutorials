cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Set the C++ standard to C++14
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

# Add include directories for standard system headers if needed
include_directories(/usr/include)
include_directories(/usr/include/c++/9)

add_executable(talker src/publisher_member_function.cpp)
add_executable(listener src/subscriber_member_function.cpp)
add_executable(test_talker test/test_talker.cpp)

ament_target_dependencies(talker rclcpp std_msgs example_interfaces geometry_msgs rclcpp tf2 tf2_ros)
ament_target_dependencies(listener rclcpp std_msgs example_interfaces geometry_msgs rclcpp tf2 tf2_ros)
ament_target_dependencies(test_talker rclcpp std_msgs tf2_ros)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

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

ament_package()