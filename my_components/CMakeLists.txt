cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(generic_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

add_library(pre_approach_component SHARED src/pre_approach_component.cpp)
target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_component
	"rclcpp"
	"rclcpp_components"
	"geometry_msgs"
	nav_msgs
	sensor_msgs
	generic_manager
)
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach_component>\n")

add_library(server_component SHARED src/attach_server_component.cpp)
target_compile_definitions(server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "generic_manager"

)
rclcpp_components_register_nodes(server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:server_component>\n")

add_library(client_component SHARED src/attach_client_component.cpp)
target_compile_definitions(client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(client_component
  "rclcpp"
  "rclcpp_components"
	nav_msgs
	sensor_msgs
	generic_manager
	geometry_msgs
	tf2_geometry_msgs
	std_msgs
)
rclcpp_components_register_nodes(client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:client_component>\n")



add_executable(pre_approach_and_server
  
  src/pre_approach_and_server.cpp)
target_link_libraries(pre_approach_and_server
  pre_approach_component
  server_component)

ament_target_dependencies(pre_approach_and_server
  "rclcpp")



rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
   DEPENDENCIES std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(server_component
  ${cpp_typesupport_target})

target_link_libraries(client_component
  ${cpp_typesupport_target})





install(TARGETS
  pre_approach_and_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pre_approach_component
  server_component
  client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Install launch files.
install(DIRECTORY
  launch
  rviz
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
