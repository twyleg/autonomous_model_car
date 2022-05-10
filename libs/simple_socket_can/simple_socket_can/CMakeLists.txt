set(TARGET_NAME simple_socket_can)

#
# set cmake settings
#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# add source files to target
#
add_library(${TARGET_NAME}
	socket_can.cc
	socket_can.h
)

#
# add parent folder to targets include directories
#
target_include_directories(${TARGET_NAME}
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
)
