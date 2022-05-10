# Copyright (C) 2021 twyleg
import rclpy
from os import set_blocking
from typing import BinaryIO, AnyStr, Optional
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rcl_interfaces.msg import ParameterDescriptor
from sony_dualshock_three_controller_interfaces.msg import SonyDualShockThreeControllerInputRaw
from sony_dualshock_three_controller_interfaces.msg import SonyDualShockThreeControllerInputPercentage


class SonyDualShockThreeController(object):
	hidraw_file_path: str
	hidraw_file: BinaryIO
	data: Optional[AnyStr]

	def __init__(self, hidraw_file_path: str) -> None:
		self.hidraw_file_path = hidraw_file_path
		self.hidraw_file = open(self.hidraw_file_path, 'rb')
		set_blocking(self.hidraw_file.fileno(), False)
		self.data = None

	def read_data(self) -> None:
		while True:
			data = self.hidraw_file.read(49)
			if data is not None:
				self.data = data
			else:
				break

	def get_analog_stick_left_x(self) -> int:
		return self.data[6]

	def get_analog_stick_left_y(self) -> int:
		return self.data[7]

	def get_analog_stick_right_x(self) -> int:
		return self.data[8]

	def get_analog_stick_right_y(self) -> int:
		return self.data[9]

	def get_button_l1(self) -> int:
		return self.data[20]

	def get_button_l2(self) -> int:
		return self.data[18]

	def get_button_r1(self) -> int:
		return self.data[21]

	def get_button_r2(self) -> int:
		return self.data[19]

	def get_button_cross(self) -> int:
		return self.data[24]

	def get_button_square(self) -> int:
		return self.data[25]

	def get_button_triangle(self) -> int:
		return self.data[22]

	def get_button_circle(self) -> int:
		return self.data[23]

	def get_cross_up(self) -> int:
		return self.data[14]

	def get_cross_down(self) -> int:
		return self.data[16]

	def get_cross_left(self) -> int:
		return self.data[17]

	def get_cross_right(self) -> int:
		return self.data[15]

	def data_valid(self) -> bool:
		return self.data is not None


class SonyDualshockThreeControllerPublisher(Node):

	PUBLISHER_RAW_TOPIC = '/hid/SonyDualShockThreeControllerInputRaw'
	PUBLISHER_PERCENTAGE_TOPIC = '/hid/SonyDualShockThreeControllerInputPercentage'

	publisher_raw: Publisher
	publisher_percentage: Publisher
	timer: Timer
	controller: SonyDualShockThreeController

	@staticmethod
	def apply_analog_stick_offset(input_value: int) -> int:
		return input_value - 128;

	@staticmethod
	def analog_stick_value_to_percentage(input_value: int) -> float:
		return (input_value - 128) / 127.0;

	@staticmethod
	def button_value_to_percentage(input_value: int) -> float:
		return input_value / 255.0;

	def __init__(self) -> None:
		super().__init__('sony_dualshock_three_controller_publisher')

		self.publisher_raw = self.create_publisher(SonyDualShockThreeControllerInputRaw,
												   SonyDualshockThreeControllerPublisher.PUBLISHER_RAW_TOPIC, 10)
		self.publisher_percentage = self.create_publisher(SonyDualShockThreeControllerInputPercentage,
														  SonyDualshockThreeControllerPublisher.PUBLISHER_PERCENTAGE_TOPIC,
														  10)

		controller_hidraw_device_path_parameter_descriptor = ParameterDescriptor(description='Hidraw device path for '
																							 'Sony Dualshock 3 '
																							 'controller, e.g. '
																							 '/dev/hidraw0')
		self.declare_parameter('controller_hidraw_device_path', rclpy.Parameter.Type.STRING,
							   controller_hidraw_device_path_parameter_descriptor)
		controller_hidraw_device_path = self.get_parameter('controller_hidraw_device_path').get_parameter_value().string_value
		self.controller = SonyDualShockThreeController(controller_hidraw_device_path)
		self.get_logger().info('Using controller with hidraw device path "%s"' % controller_hidraw_device_path)

		publisher_period_parameter_descriptor = ParameterDescriptor(description='The period in seconds with which the '
																				'controller input will be published')
		self.declare_parameter('publisher_period', 0.1, publisher_period_parameter_descriptor)
		publisher_period = self.get_parameter('publisher_period').get_parameter_value().double_value
		self.timer = self.create_timer(publisher_period, self.timer_callback)
		self.get_logger().info('Starting to publish with period %fs' % publisher_period)

	def timer_callback(self) -> None:
		self.controller.read_data()
		if self.controller.data_valid():
			self.publish_controller_input_raw()
			self.publish_controller_input_percentage()

	def publish_controller_input_raw(self):
		controller_input_raw = SonyDualShockThreeControllerInputRaw()

		controller_input_raw.analog_stick_left.x = self.apply_analog_stick_offset(self.controller.get_analog_stick_left_x())
		controller_input_raw.analog_stick_left.y = self.apply_analog_stick_offset(self.controller.get_analog_stick_left_y())
		controller_input_raw.analog_stick_right.x = self.apply_analog_stick_offset(self.controller.get_analog_stick_right_x())
		controller_input_raw.analog_stick_right.y = self.apply_analog_stick_offset(self.controller.get_analog_stick_right_y())
		controller_input_raw.button_l1 = self.controller.get_button_l1()
		controller_input_raw.button_l2 = self.controller.get_button_l2()
		controller_input_raw.button_r1 = self.controller.get_button_r1()
		controller_input_raw.button_r2 = self.controller.get_button_r2()
		controller_input_raw.button_cross = self.controller.get_button_cross()
		controller_input_raw.button_square = self.controller.get_button_square()
		controller_input_raw.button_triangle = self.controller.get_button_triangle()
		controller_input_raw.button_circle = self.controller.get_button_circle()
		controller_input_raw.cross_up = self.controller.get_cross_up()
		controller_input_raw.cross_down = self.controller.get_cross_down()
		controller_input_raw.cross_left = self.controller.get_cross_left()
		controller_input_raw.cross_right = self.controller.get_cross_right()

		self.publisher_raw.publish(controller_input_raw)

	def publish_controller_input_percentage(self):
		controller_input_percentage = SonyDualShockThreeControllerInputPercentage()

		controller_input_percentage.analog_stick_left.x = self.analog_stick_value_to_percentage(self.controller.get_analog_stick_left_x())
		controller_input_percentage.analog_stick_left.y = self.analog_stick_value_to_percentage(self.controller.get_analog_stick_left_y())
		controller_input_percentage.analog_stick_right.x = self.analog_stick_value_to_percentage(self.controller.get_analog_stick_right_x())
		controller_input_percentage.analog_stick_right.y = self.analog_stick_value_to_percentage(self.controller.get_analog_stick_right_y())
		controller_input_percentage.button_l1 = self.button_value_to_percentage(self.controller.get_button_l1())
		controller_input_percentage.button_l2 = self.button_value_to_percentage(self.controller.get_button_l2())
		controller_input_percentage.button_r1 = self.button_value_to_percentage(self.controller.get_button_r1())
		controller_input_percentage.button_r2 = self.button_value_to_percentage(self.controller.get_button_r2())
		controller_input_percentage.button_cross = self.button_value_to_percentage(self.controller.get_button_cross())
		controller_input_percentage.button_square = self.button_value_to_percentage(self.controller.get_button_square())
		controller_input_percentage.button_triangle = self.button_value_to_percentage(self.controller.get_button_triangle())
		controller_input_percentage.button_circle = self.button_value_to_percentage(self.controller.get_button_circle())
		controller_input_percentage.cross_up = self.button_value_to_percentage(self.controller.get_cross_up())
		controller_input_percentage.cross_down = self.button_value_to_percentage(self.controller.get_cross_down())
		controller_input_percentage.cross_left = self.button_value_to_percentage(self.controller.get_cross_left())
		controller_input_percentage.cross_right = self.button_value_to_percentage(self.controller.get_cross_right())

		self.publisher_percentage.publish(controller_input_percentage)


def main(args=None):
	rclpy.init(args=args)

	publisher = SonyDualshockThreeControllerPublisher()

	rclpy.spin(publisher)

	publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
