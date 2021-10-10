from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sensor_grabber',
			executable='sensor_grabber'
		),
		Node(
			package='environment_recognition',
			executable='environment_recognition'
		),
		Node(
			package='secondary_vehicle_remote_control',
			executable='secondary_vehicle_remote_control',
			parameters=[
				{"velocity": 0.0}
			]
		)
	])
