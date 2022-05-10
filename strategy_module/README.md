# ROS Workspace

## Prerequisites

Packages:

* ros2 (tested with galactic)
* Gazebo Simulator (tested with 11.8.1)
* gazebo_ros_pkgs

## Usage

Source ROS installation:

	source /opt/ros2/galactic/setup.bash

Build:

	colcon build

Source current ROS workspace:

	source install/local_setup.bash

Run with launch file:

	 ros2 launch launch/acc_drive_gazebo_launch.py

Control secondary vehicle (the vehicle we're following) by replaying input data from file:

	ros2 bag play bags/secondary_vehicle_remote_control_input_constant_drive

## References

[http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
[https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp)
