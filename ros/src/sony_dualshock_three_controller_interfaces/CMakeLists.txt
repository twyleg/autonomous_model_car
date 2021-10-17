cmake_minimum_required(VERSION 3.8)
project(sony_dualshock_three_controller_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/Vector2.msg"
	"msg/SonyDualShockThreeControllerInput.msg"
)

ament_package()
