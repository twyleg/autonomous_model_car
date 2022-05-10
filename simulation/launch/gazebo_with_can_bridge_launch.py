from launch import LaunchDescription, actions
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='can_bridge',
			executable='can_bridge',
			parameters=[
				{'can_interface': 'vcan1'}
			],
			remappings=[
			]
		),
		actions.ExecuteProcess(
			cmd=['gazebo', '--verbose', 'gazebo/worlds/follow_drive.world'],
			output='screen'
		)
	])
