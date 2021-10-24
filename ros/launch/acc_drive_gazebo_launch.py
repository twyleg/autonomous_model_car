from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sony_dualshock_three_controller_publisher',
			executable='sony_dualshock_three_controller_publisher',
			parameters=[
				{'controller_hidraw_device_path': '/dev/sony_dualshockthree'}
			],
			remappings=[
				('/hid/SonyDualShockThreeControllerInputPercentage', '/primary_vehicle/vehicle_remote_control/input/SonyDualShockThreeControllerInputPercentage'),
			]
		),
		Node(
			package='vehicle_remote_control',
			executable='vehicle_remote_control',
			name='secondary_vehicle_remote_control',
			remappings=[
				('/vehicle_remote_control/input/SonyDualShockThreeControllerInputPercentage', '/secondary_vehicle/vehicle_remote_control/input/SonyDualShockThreeControllerInputPercentage'),
				('/vehicle_remote_control/output/cmd_vel', '/secondary_vehicle/cmd_vel'),
			]
		),
		Node(
			package='vehicle_abstraction_layer',
			executable='vehicle_abstraction_layer_gazebo'
		),
		Node(
			package='environment_recognition',
			executable='environment_recognition'
		),
		Node(
			package='drive_strategies',
			executable='acc',
		),
	])
