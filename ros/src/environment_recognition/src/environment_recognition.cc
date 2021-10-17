
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/sensor_data.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/vehicle_state.hpp"
#include "environment_recognition_interfaces/msg/target_vehicle.hpp"

#include <memory>
#include <optional>
#include <utility>
#include <chrono>
#include <ratio>

using std::placeholders::_1;

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

	}

private:

	void sensorDataCallback(const vehicle_abstraction_layer_interfaces::msg::SensorData::SharedPtr msg) {

		auto targetVehicle = environment_recognition_interfaces::msg::TargetVehicle();

		const float ultrasoundDistance = msg->ultrasound_distance;
		const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

		if (mLastUltrasoundDistance == std::nullopt) {
			mLastUltrasoundDistance = std::make_pair(now, ultrasoundDistance);
			targetVehicle.set__available(false);
		} else if (mVehicleState == std::nullopt) {
			targetVehicle.set__available(false);
		} else if (ultrasoundDistance > 40.0) {
			targetVehicle.set__available(false);
		} else {
			const std::chrono::duration<double> dTime = now - mLastUltrasoundDistance->first;
			const float dDistance = ultrasoundDistance - mLastUltrasoundDistance->second;
			const float ownVelocity = mVehicleState->twist.linear.x;
			const float targetRelativeVelocity = dDistance / dTime.count();
			const float targetAbsoluteVelocity = targetRelativeVelocity + ownVelocity;

			RCLCPP_INFO(this->get_logger(), "ultrasoundDistance: '%f'", ultrasoundDistance);
			RCLCPP_INFO(this->get_logger(), "dTime '%f'", dTime.count());
			RCLCPP_INFO(this->get_logger(), "dDistance: '%f'", dDistance);
			RCLCPP_INFO(this->get_logger(), "rel v: '%f'", targetRelativeVelocity);
			RCLCPP_INFO(this->get_logger(), "abs v: '%f'", targetAbsoluteVelocity);

			mLastUltrasoundDistance = std::make_pair(now, ultrasoundDistance);

			targetVehicle.set__absolute_velocity(targetAbsoluteVelocity);
			targetVehicle.set__relative_velocity(targetRelativeVelocity);
			targetVehicle.set__distance(ultrasoundDistance);
			targetVehicle.set__available(true);
		}

		mTargetVehiclePublisher->publish(targetVehicle);

	}

	void vehicleStateCallback(const vehicle_abstraction_layer_interfaces::msg::VehicleState::SharedPtr msg) {
		mVehicleState = *msg;
	}

	rclcpp::Subscription<vehicle_abstraction_layer_interfaces::msg::SensorData>::SharedPtr mSensorDataSubscription;
	rclcpp::Subscription<vehicle_abstraction_layer_interfaces::msg::VehicleState>::SharedPtr mVehicleStateSubscription;

	rclcpp::Publisher<environment_recognition_interfaces::msg::TargetVehicle>::SharedPtr mTargetVehiclePublisher;

	std::optional<vehicle_abstraction_layer_interfaces::msg::VehicleState> mVehicleState;
	std::optional<std::pair<std::chrono::time_point<std::chrono::system_clock>, float>> mLastUltrasoundDistance;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EnvironmentRecognition>());
	rclcpp::shutdown();
	return 0;
}
