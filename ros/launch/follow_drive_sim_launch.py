from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sony_dualshock_three_controller_publisher',
			executable='sony_dualshock_three_controller_publisher',
			name='sony_dualshock_three_controller_publisher',
			output='screen',
			emulate_tty=True,
			parameters=[
				{'controller_hidraw_device_path': '/dev/sony_dualshockthree'}
			],
			remappings=[
				('/hid/SonyDualShockThreeControllerInput', '/primary_vehicle/vehicle_remote_control/input/SonyDualShockThreeControllerInput'),
			]
		),
		Node(
			package='vehicle_remote_control',
			executable='vehicle_remote_control',
			name='secondary_vehicle_remote_control',
			output='screen',
			emulate_tty=True,
			remappings=[
				('/vehicle_remote_control/input/SonyDualShockThreeControllerInput', '/secondary_vehicle/vehicle_remote_control/input/SonyDualShockThreeControllerInput'),
				('/vehicle_remote_control/output/cmd_vel', '/secondary_vehicle/cmd_vel'),
			]
		),
		Node(
			package='vehicle_abstraction_layer',
			executable='vehicle_abstraction_layer_virtual'
		),
		Node(
			package='environment_recognition',
			executable='environment_recognition'
		),
	])
