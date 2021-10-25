cmake_minimum_required(VERSION 3.8)
project(vehicle_abstraction_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vehicle_abstraction_layer_interfaces REQUIRED)

add_executable(vehicle_abstraction_layer_gazebo
	src/vehicle_abstraction_layer_gazebo.cc
)

ament_target_dependencies(vehicle_abstraction_layer_gazebo
	rclcpp
	std_msgs
	sensor_msgs
	nav_msgs
	vehicle_abstraction_layer_interfaces
)

add_executable(vehicle_abstraction_layer_can
	src/vehicle_abstraction_layer_can.cc
	src/socket_can.h
	src/socket_can.cc
)

ament_target_dependencies(vehicle_abstraction_layer_can
	rclcpp
	std_msgs
	sensor_msgs
	nav_msgs
	vehicle_abstraction_layer_interfaces
)

install(
	TARGETS
		vehicle_abstraction_layer_gazebo
		vehicle_abstraction_layer_can
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
