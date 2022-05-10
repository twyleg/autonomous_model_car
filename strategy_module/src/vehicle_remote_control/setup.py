from setuptools import setup

package_name = 'vehicle_remote_control'

setup(
	name=package_name,
	version='0.0.1',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='twyleg',
	maintainer_email='mail@twyleg.de',
	description='Vehicle remote control for Sony DualShock 3 controller and gazebo ackermann vehicle model',
	license='GPL-3.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'vehicle_remote_control = vehicle_remote_control.vehicle_remote_control:main',
		],
	},
)
