cmake_minimum_required(VERSION 3.5)
project(ros2_imu_filter)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Other Packages
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(imu_node
  src/imu_factory.cpp
  src/imu_base_node.cpp
  src/imu_complementary_node.cpp
  src/imu_node.cpp
  src/kalman_filter.cpp
)

ament_target_dependencies(imu_node
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  Eigen3
)

install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS imu_node
  RUNTIME DESTINATION bin
)

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

ament_package()
