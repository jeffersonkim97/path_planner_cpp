cmake_minimum_required(VERSION 3.8)
project(path_planner_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(node_publisher src/node_publisher.cpp src/rrt.cpp)
ament_target_dependencies(node_publisher rclcpp std_msgs visualization_msgs)

add_executable(node_publisher_RRTC src/node_publisher_RRTC.cpp src/rrtc.cpp)
ament_target_dependencies(node_publisher_RRTC rclcpp std_msgs visualization_msgs)

add_executable(node_publisher_RRTC3D src/node_publisher_RRTC3D.cpp src/rrtc3D.cpp)
ament_target_dependencies(node_publisher_RRTC3D rclcpp std_msgs visualization_msgs)


# add_executable(node_publisher_RRTP src/node_publisher_RRTP.cpp src/rrtp.cpp)
# ament_target_dependencies(node_publisher_RRTP rclcpp std_msgs visualization_msgs)

install(TARGETS
  node_publisher
  node_publisher_RRTC
  node_publisher_RRTC3D
  # node_publisher_RRTP
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
