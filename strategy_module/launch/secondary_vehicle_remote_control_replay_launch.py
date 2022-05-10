from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='vehicle_remote_control',
			executable='vehicle_remote_control',
			name='vehicle_remote_control',
			output='screen',
			emulate_tty=True,
			remappings=[
				('/vehicle_remote_control/input/SonyDualShockThreeControllerInput', '/secondary_vehicle/vehicle_remote_control/input/SonyDualShockThreeControllerInput'),
				('/vehicle_remote_control/output/cmd_vel', '/secondary_vehicle_control/cmd_vel'),
			]
		),
	])

