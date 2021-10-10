cmake_minimum_required(VERSION 3.8)
project(sensor_grabber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sensor_grabber
	src/sensor_grabber.cc
)

ament_target_dependencies(sensor_grabber
	rclcpp
	std_msgs
	sensor_msgs
)

install(
	TARGETS
		sensor_grabber
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
