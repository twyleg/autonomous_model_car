# Gazebo

## Prerequisites

Source the gazebo environment:

	source /usr/share/gazebo/setup.sh

Install **gazebo_ros_pkgs** for **ROS2** and make sure it is sourced:

	source gazebo_ros_pkgs/install/setup.bash

## Usage

Start up gazebo with the roundcourse world in pause mode:

	gazebo worlds/follow_drive.world -u --verbose
	
## References

* [http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
* [https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_plugins/worlds/gazebo_ros_ackermann_drive_demo.world](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_plugins/worlds/gazebo_ros_ackermann_drive_demo.world) 