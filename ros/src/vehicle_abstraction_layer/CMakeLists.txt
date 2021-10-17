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

add_executable(vehicle_abstraction_layer_virtual
	src/vehicle_interface_virtual.cc
)

ament_target_dependencies(vehicle_abstraction_layer_virtual
	rclcpp
	std_msgs
	sensor_msgs
	nav_msgs
	vehicle_abstraction_layer_interfaces
)

install(
	TARGETS
		vehicle_abstraction_layer_virtual
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
