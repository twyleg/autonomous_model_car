from setuptools import setup

package_name = 'sony_dualshock_three_controller_publisher'

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
    description='Publisher for Sony DualShock 3 Controller input data',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sony_dualshock_three_controller_publisher = sony_dualshock_three_controller_publisher.publisher:main',
        ],
    },
)
