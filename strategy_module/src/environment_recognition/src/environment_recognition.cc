// Copyright (C) 2021 twyleg
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/sensor_data.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/vehicle_state.hpp"
#include "environment_recognition_interfaces/msg/target_vehicle.hpp"

#include <memory>
#include <optional>
#include <utility>
#include <chrono>
#include <ratio>

using std::placeholders::_1;

using namespace std::chrono_literals;

class EnvironmentRecognition : public rclcpp::Node
{
public:
	EnvironmentRecognition() :
		Node("environment_recognition")
	{
		mSensorDataSubscription = this->create_subscription<vehicle_abstraction_layer_interfaces::msg::SensorData>(
				"/vehicle_abstraction_layer/output/sensor_data", 10, std::bind(&EnvironmentRecognition::sensorDataCallback, this, _1));

		mVehicleStateSubscription = this->create_subscription<vehicle_abstraction_layer_interfaces::msg::VehicleState>(
				"/vehicle_abstraction_layer/output/vehicle_state", 10, std::bind(&EnvironmentRecognition::vehicleStateCallback, this, _1));

		mTargetVehiclePublisher = this->create_publisher<environment_recognition_interfaces::msg::TargetVehicle>(
				"/environment_recognition/output/target_vehicle", 10);

		mMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>(
				"visualization_marker", 1);

	}

private:

	void sensorDataCallback(const vehicle_abstraction_layer_interfaces::msg::SensorData::SharedPtr msg) {

		auto targetVehicle = environment_recognition_interfaces::msg::TargetVehicle();

		const float ultrasoundDistance = msg->ultrasound_distance;
		const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

		if (mLastUltrasoundDistance == std::nullopt) {
			mLastUltrasoundDistance = std::make_pair(now, ultrasoundDistance);
			targetVehicle.set__available(false);
		} else if (ultrasoundDistance > 40.0) {
			targetVehicle.set__available(false);
			mLastUltrasoundDistance = std::nullopt;
		} else if (mVehicleState == std::nullopt) {
			targetVehicle.set__available(false);
		} else {
			const std::chrono::duration<double> dTime = now - mLastUltrasoundDistance->first;
			const float dDistance = ultrasoundDistance - mLastUltrasoundDistance->second;
			const float ownVelocity = mVehicleState->twist.linear.x;
			const float targetRelativeVelocity = dDistance / dTime.count();
			const float targetAbsoluteVelocity = targetRelativeVelocity + ownVelocity;

			targetVehicle.set__absolute_velocity(targetAbsoluteVelocity);
			targetVehicle.set__relative_velocity(targetRelativeVelocity);
			targetVehicle.set__distance(ultrasoundDistance);
			targetVehicle.set__available(true);

			mLastUltrasoundDistance = std::make_pair(now, ultrasoundDistance);
		}

		mTargetVehiclePublisher->publish(targetVehicle);

		auto marker = visualization_msgs::msg::Marker();

		marker.header.frame_id = "/my_frame";
		marker.header.stamp = this->get_clock()->now();

		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.type = visualization_msgs::msg::Marker::CUBE;

		marker.action = visualization_msgs::msg::Marker::ADD;

		marker.pose.position.x = ultrasoundDistance;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;

		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = rclcpp::Duration(1s);

		mMarkerPublisher->publish(marker);

	}

	void vehicleStateCallback(const vehicle_abstraction_layer_interfaces::msg::VehicleState::SharedPtr msg) {
		mVehicleState = *msg;
	}

	rclcpp::Subscription<vehicle_abstraction_layer_interfaces::msg::SensorData>::SharedPtr mSensorDataSubscription;
	rclcpp::Subscription<vehicle_abstraction_layer_interfaces::msg::VehicleState>::SharedPtr mVehicleStateSubscription;

	rclcpp::Publisher<environment_recognition_interfaces::msg::TargetVehicle>::SharedPtr mTargetVehiclePublisher;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mMarkerPublisher;

	std::optional<vehicle_abstraction_layer_interfaces::msg::VehicleState> mVehicleState;
	std::optional<std::pair<std::chrono::time_point<std::chrono::system_clock>, float>> mLastUltrasoundDistance;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EnvironmentRecognition>());
	rclcpp::shutdown();
	return 0;
}
