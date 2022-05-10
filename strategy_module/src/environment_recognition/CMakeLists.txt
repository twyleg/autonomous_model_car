cmake_minimum_required(VERSION 3.8)
project(environment_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vehicle_abstraction_layer_interfaces REQUIRED)
find_package(environment_recognition_interfaces REQUIRED)

add_executable(environment_recognition
	src/environment_recognition.cc
)

ament_target_dependencies(environment_recognition
	rclcpp
	std_msgs
	geometry_msgs
	visualization_msgs
	vehicle_abstraction_layer_interfaces
	environment_recognition_interfaces
)

install(
	TARGETS
		environment_recognition
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
