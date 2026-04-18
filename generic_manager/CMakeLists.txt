cmake_minimum_required(VERSION 3.8)
project(generic_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# =========================
# diff_drive_manager
# =========================
add_library(diff_drive_manager
  src/diff_drive_manager.cpp
)

ament_target_dependencies(diff_drive_manager
  rclcpp
  geometry_msgs
  rclcpp_lifecycle
)

target_include_directories(diff_drive_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# laser_manager
# =========================
add_library(laser_manager
  src/laser_manager.cpp
)

ament_target_dependencies(laser_manager
  rclcpp
  sensor_msgs
  rclcpp_lifecycle
)

target_include_directories(laser_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# odom_manager
# =========================
add_library(odom_manager
  src/odom_manager.cpp
)

ament_target_dependencies(odom_manager
  rclcpp
  nav_msgs
  rclcpp_lifecycle
  tf2
)

target_include_directories(odom_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# lifecycle_manager
# =========================
add_library(lifecycle_manager
  src/lifecycle_manager.cpp
)

ament_target_dependencies(lifecycle_manager
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
)

target_include_directories(lifecycle_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# service_manager
# =========================
add_library(service_manager
  src/ros2_service_manager.cpp
)

ament_target_dependencies(service_manager
	rclcpp
)

target_include_directories(service_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# cpp_helper
# =========================
add_library(cpp_helper
  src/cpp_helper.cpp
)

ament_target_dependencies(cpp_helper
)

target_include_directories(cpp_helper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# robo_math
# =========================
add_library(robo_math
  src/robo_math.cpp
)

ament_target_dependencies(robo_math
)

target_include_directories(robo_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



# =========================
# install libraries
# =========================
install(TARGETS
  diff_drive_manager
  laser_manager
  odom_manager
  lifecycle_manager
  service_manager
  cpp_helper
  robo_math
  EXPORT export_generic_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install headers
install(DIRECTORY include/
  DESTINATION include
)

# export
ament_export_targets(export_generic_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  rclcpp_lifecycle
  sensor_msgs
)

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()