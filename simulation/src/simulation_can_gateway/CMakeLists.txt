cmake_minimum_required(VERSION 3.8)
project(simulation_can_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_subdirectory(libs/simple_socket_can/)

add_executable(simulation_can_gateway
	src/simulation_can_gateway.cc
)

target_link_libraries(simulation_can_gateway
	simple_socket_can
)

ament_target_dependencies(simulation_can_gateway
	rclcpp
	std_msgs
	sensor_msgs
	nav_msgs
)

install(
	TARGETS
		simulation_can_gateway
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
