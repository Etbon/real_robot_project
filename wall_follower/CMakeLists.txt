cmake_minimum_required(VERSION 3.5)
project(wall_follower)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

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


add_executable(wall_follower_node src/wall_follower.cpp)
add_executable(find_wall_service_node src/find_wall_service.cpp)
add_executable(odom_record_action_server_node src/odom_record_action_server.cpp)

ament_target_dependencies(wall_follower_node 
	rclcpp 
	rclcpp_action
	geometry_msgs 
	sensor_msgs
	custom_interfaces
)

ament_target_dependencies(find_wall_service_node 
	rclcpp 
	sensor_msgs 
	geometry_msgs 
	rosidl_default_generators
	std_srvs
	custom_interfaces
)

ament_target_dependencies(odom_record_action_server_node
	rclcpp
	rclcpp_action
	sensor_msgs
	nav_msgs
	geometry_msgs
	rosidl_default_generators
	custom_interfaces

)


install(TARGETS
	wall_follower_node
	find_wall_service_node
	odom_record_action_server_node
	DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()