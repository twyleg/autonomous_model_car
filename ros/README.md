# ROS Workspace

## Prerequisites

Packages:

* gazebo_ros_pkgs

## Usage

Source ROS installation:

	bash
	source /opt/ros2/galactic/setup.bash

Build:

	colcon build
	
Source current ROS workspace:

	source install/local_setup.bash
	
Run with launch file:

	 ros2 launch launch/piracer_acc_sim_launch.py
	 
Set speed of secondary vehicle (the vehicle we're following):

	ros2 param set /secondary_vehicle_remote_control velocity 1.0
		
## References

[http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)