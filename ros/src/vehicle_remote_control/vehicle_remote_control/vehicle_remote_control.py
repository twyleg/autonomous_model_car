# Copyright (C) 2021 twyleg
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sony_dualshock_three_controller_interfaces.msg import SonyDualShockThreeControllerInputPercentage

class VehicleRemoteControl(Node):

	def __init__(self):
		super().__init__('vehicle_remote_control')
		self.subscription = self.create_subscription(
			SonyDualShockThreeControllerInputPercentage,
			'/vehicle_remote_control/input/SonyDualShockThreeControllerInputPercentage',
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

		self.publisher = self.create_publisher(Twist, '/vehicle_remote_control/output/cmd_vel', 10)

	def listener_callback(self, msg):
		twist = Twist()
		twist.linear.x = 20.0 * (-msg.analog_stick_right.y)
		twist.angular.z = 0.6458 * (-msg.analog_stick_left.x)
		self.publisher.publish(twist)


def main(args=None):
	rclpy.init(args=args)

	remote_control = VehicleRemoteControl()

	rclpy.spin(remote_control)

	remote_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
