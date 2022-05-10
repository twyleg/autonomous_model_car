# Gazebo

## Prerequisites

Source the gazebo environment:

	source /usr/share/gazebo/setup.sh

Install **gazebo_ros_pkgs** for **ROS2** and make sure it is sourced:

	source gazebo_ros_pkgs/install/setup.bash

Source ros workspace:

	source install/setup.bash


## Setup virtual CAN

Load can-gw module

	modprobe can-gw	

Setup virtual CAN devices and routing

	ip link add dev vcan0 type vcan
	ip link add dev vcan1 type vcan
	ip link set up vcan0
	ip link set up vcan1
	cangw -A -s vcan0 -d vcan1 -e
	cangw -A -s vcan1 -d vcan0 -e

## Usage

Start up gazebo with the roundcourse world in pause mode:

	gazebo worlds/follow_drive.world -u --verbose
	
## References

* [http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
* [https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_plugins/worlds/gazebo_ros_ackermann_drive_demo.world](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_plugins/worlds/gazebo_ros_ackermann_drive_demo.world) 
