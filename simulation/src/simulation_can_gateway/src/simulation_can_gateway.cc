// Copyright (C) 2021 twyleg
#include "simple_socket_can/socket_can.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>


using std::placeholders::_1;

using namespace std::chrono_literals;

class SimulationCanGateway : public rclcpp::Node
{
public:
	SimulationCanGateway() :
		Node("simulation_can_gateway"),
		mSocketCAN(this->declare_parameter<std::string>("can_interface"))
	{
		mSocketCAN.setBlocking(false);

		mUltrasoundSensorSubscription = this->create_subscription<sensor_msgs::msg::Range>(
				"/primary_vehicle/ultrasound/range", rclcpp::SensorDataQoS(),
				std::bind(&SimulationCanGateway::ultrasoundSensorCallback, this, _1));

		mOdometrySubscription = this->create_subscription<nav_msgs::msg::Odometry>(
				"/primary_vehicle/odom", rclcpp::SensorDataQoS(),
				std::bind(&SimulationCanGateway::odometryCallback, this, _1));

		mDistanceSubscription = this->create_subscription<std_msgs::msg::Float32>(
				"/primary_vehicle/distance", rclcpp::SensorDataQoS(),
				std::bind(&SimulationCanGateway::distanceCallback, this, _1));


		mDriveControlPublisher = this->create_publisher<geometry_msgs::msg::Twist>(
				"/primary_vehicle/cmd_vel", 10);


		mTimer = this->create_wall_timer(1000ms, std::bind(&SimulationCanGateway::timerCallback, this));
	}

private:

	void ultrasoundSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
		mUltrasoundSensorDistance = msg->range;
	}

	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		mOdometry = *msg;
	}

	void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
		mDistance = msg->data;
	}

	void timerCallback() {

		RCLCPP_INFO(this->get_logger(), "Distance %f", mUltrasoundSensorDistance);

		can_frame driveCommandCanFrame;
		if (mSocketCAN.read(driveCommandCanFrame)) {
			RCLCPP_INFO(this->get_logger(), "CAN received");

//			float targetVelocity;
//			float targetYawRate;

//			auto driveCommandTwist = geometry_msgs::msg::Twist();
//			driveCommandTwist.linear.set__x(targetVelocity);
//			driveCommandTwist.angular.set__z(targetYawRate);
//			mDriveControlPublisher->publish(driveCommandTwist);

		}

		can_frame sensorDataCanFrame;

		sensorDataCanFrame.len = 1;
		sensorDataCanFrame.data[0] = 42;
		mSocketCAN.write(sensorDataCanFrame);
	}

	float mDistance;
	float mUltrasoundSensorDistance;
	nav_msgs::msg::Odometry mOdometry;

	SocketCAN mSocketCAN;

	// Gazebo -> CAN
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr mUltrasoundSensorSubscription;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdometrySubscription;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mDistanceSubscription;

	// CAN -> Gazebo
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mDriveControlPublisher;

	rclcpp::TimerBase::SharedPtr mTimer;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimulationCanGateway>());
	rclcpp::shutdown();
	return 0;
}
