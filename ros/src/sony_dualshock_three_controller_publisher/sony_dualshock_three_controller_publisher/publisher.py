# Copyright (C) 2021 twyleg
import rclpy
from rclpy.node import Node

from sony_dualshock_three_controller_interfaces.msg import SonyDualShockThreeControllerInput

class SonyDualShockThreeController(object):

	def __init__(self, hidrawFilePath):
		self.hidrawFilePath = hidrawFilePath
		self.hidrawFile = open(self.hidrawFilePath, 'rb')
		self.data = None

	def read_data(self):
		self.data = self.hidrawFile.read(49)

	def get_analog_stick_left_x(self):
		return self.data[6]

	def get_analog_stick_left_y(self):
		return self.data[7]

	def get_analog_stick_right_x(self):
		return self.data[8]

	def get_analog_stick_right_y(self):
		return self.data[9]

	def get_button_l1(self):
		return self.data[20]

	def get_button_l2(self):
		return self.data[18]

	def get_button_r1(self):
		return self.data[21]

	def get_button_r2(self):
		return self.data[19]

	def get_button_cross(self):
		return self.data[24]

	def get_button_square(self):
		return self.data[25]

	def get_button_triangle(self):
		return self.data[22]

	def get_button_circle(self):
		return self.data[23]

	def get_cross_up(self):
		return self.data[14]

	def get_cross_down(self):
		return self.data[16]

	def get_cross_left(self):
		return self.data[17]

	def get_cross_right(self):
		return self.data[15]


class Publisher(Node):

	def __init__(self):
		super().__init__('sony_dualshock_three_controller_publisher')
		self.publisher_ = self.create_publisher(SonyDualShockThreeControllerInput, '/hid/SonyDualShockThreeControllerInput', 10)

		self.declare_parameter('controller_hidraw_device_path', rclpy.Parameter.Type.STRING)
		controller_hidraw_device_path = self.get_parameter('controller_hidraw_device_path').get_parameter_value().string_value

		self.controller = SonyDualShockThreeController(controller_hidraw_device_path)

	def publish_controller_data_synchronous(self):

		self.controller.read_data()
		controllerInput = SonyDualShockThreeControllerInput()

		controllerInput.analog_stick_left.x = self.controller.get_analog_stick_left_x() - 128
		controllerInput.analog_stick_left.y = self.controller.get_analog_stick_left_y() - 128
		controllerInput.analog_stick_right.x = self.controller.get_analog_stick_right_x() - 128
		controllerInput.analog_stick_right.y = self.controller.get_analog_stick_right_y() - 128
		controllerInput.button_l1 = self.controller.get_button_l1()
		controllerInput.button_l2 = self.controller.get_button_l2()
		controllerInput.button_r1 = self.controller.get_button_r1()
		controllerInput.button_r2 = self.controller.get_button_r2()
		controllerInput.button_cross = self.controller.get_button_cross()
		controllerInput.button_square = self.controller.get_button_square()
		controllerInput.button_triangle = self.controller.get_button_triangle()
		controllerInput.button_circle = self.controller.get_button_circle()
		controllerInput.cross_up = self.controller.get_cross_up()
		controllerInput.cross_down = self.controller.get_cross_down()
		controllerInput.cross_left = self.controller.get_cross_left()
		controllerInput.cross_right = self.controller.get_cross_right()

		self.publisher_.publish(controllerInput)


def main(args=None):
	rclpy.init(args=args)

	publisher = Publisher()

	while True:
		publisher.publish_controller_data_synchronous()

	publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
