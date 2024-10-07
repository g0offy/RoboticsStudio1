cmake_minimum_required(VERSION 3.8)
project(sprint3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Add this line

# include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# add executables
add_executable(overlay src/overlay.cpp)
ament_target_dependencies(overlay rclcpp std_msgs cv_bridge)
target_link_libraries(overlay ${OpenCV_LIBS})

add_executable(identify src/identify_node.cpp)
ament_target_dependencies(identify rclcpp std_msgs cv_bridge sensor_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs nav_msgs)  # Add nav_msgs here
target_link_libraries(identify ${OpenCV_LIBS})

# install targets
install(TARGETS
  overlay
  identify
  DESTINATION lib/${PROJECT_NAME})

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