// Copyright (C) 2021 twyleg
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/drive_control.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/vehicle_state.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/sensor_data.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class VehicleAbstractionLayerVirtual : public rclcpp::Node
{
public:
	VehicleAbstractionLayerVirtual() :
		Node("vehicle_abstraction_layer_virtual")
	{
		mSensorDataPublisher = this->create_publisher<vehicle_abstraction_layer_interfaces::msg::SensorData>(
				"/vehicle_abstraction_layer/output/sensor_data", 10);

		mVehicleStatePublisher = this->create_publisher<vehicle_abstraction_layer_interfaces::msg::VehicleState>(
				"/vehicle_abstraction_layer/output/vehicle_state", 10);

		mDriveControlPublisher = this->create_publisher<geometry_msgs::msg::Twist>(
				"/primary_vehicle/cmd_vel", 10);


		mDriveControlSubscription = this->create_subscription<vehicle_abstraction_layer_interfaces::msg::DriveControl>(
				"/vehicle_abstraction_layer/input/drive_control", rclcpp::SensorDataQoS(),
				std::bind(&VehicleAbstractionLayerVirtual::driveControlCallback, this, _1));

		mUltrasoundSensorSubscription = this->create_subscription<sensor_msgs::msg::Range>(
				"/primary_vehicle/ultrasound/range", rclcpp::SensorDataQoS(),
				std::bind(&VehicleAbstractionLayerVirtual::ultrasoundSensorCallback, this, _1));

		mOdometrySubscription = this->create_subscription<nav_msgs::msg::Odometry>(
				"/primary_vehicle/odom", rclcpp::SensorDataQoS(),
				std::bind(&VehicleAbstractionLayerVirtual::odometryCallback, this, _1));

		mDistanceSubscription = this->create_subscription<std_msgs::msg::Float32>(
				"/primary_vehicle/distance", rclcpp::SensorDataQoS(),
				std::bind(&VehicleAbstractionLayerVirtual::distanceCallback, this, _1));

		mTimer = this->create_wall_timer(100ms, std::bind(&VehicleAbstractionLayerVirtual::timerCallback, this));
	}

private:

	void driveControlCallback(const vehicle_abstraction_layer_interfaces::msg::DriveControl::SharedPtr msg) {
		auto twist = geometry_msgs::msg::Twist();
		twist.linear.set__x(msg->target_velocity);
		twist.angular.set__y(msg->target_steering_angle);
		mDriveControlPublisher->publish(twist);
	}

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
		auto sensorData = vehicle_abstraction_layer_interfaces::msg::SensorData();
		sensorData.set__ultrasound_distance(mUltrasoundSensorDistance);
		mSensorDataPublisher->publish(sensorData);

		auto vehicleState = vehicle_abstraction_layer_interfaces::msg::VehicleState();
		vehicleState.set__distance(mDistance);
		vehicleState.set__twist(mOdometry.twist.twist);
		mVehicleStatePublisher->publish(vehicleState);
	}


	float mDistance;
	float mUltrasoundSensorDistance;
	nav_msgs::msg::Odometry mOdometry;

	// From Gazebo
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr mUltrasoundSensorSubscription;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdometrySubscription;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mDistanceSubscription;
	// To Gazebo
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mDriveControlPublisher;

	// From drive strategy
	rclcpp::Subscription<vehicle_abstraction_layer_interfaces::msg::DriveControl>::SharedPtr mDriveControlSubscription;
	// To environment recognition
	rclcpp::Publisher<vehicle_abstraction_layer_interfaces::msg::SensorData>::SharedPtr mSensorDataPublisher;
	rclcpp::Publisher<vehicle_abstraction_layer_interfaces::msg::VehicleState>::SharedPtr mVehicleStatePublisher;


	rclcpp::TimerBase::SharedPtr mTimer;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleAbstractionLayerVirtual>());
	rclcpp::shutdown();
	return 0;
}
