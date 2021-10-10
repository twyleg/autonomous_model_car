cmake_minimum_required(VERSION 3.8)
project(secondary_vehicle_remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(secondary_vehicle_remote_control
	src/secondary_vehicle_remote_control.cc
)

ament_target_dependencies(secondary_vehicle_remote_control
	rclcpp
	std_msgs
	geometry_msgs
)

install(
	TARGETS
		secondary_vehicle_remote_control
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
