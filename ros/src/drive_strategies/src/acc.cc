// Copyright (C) 2021 twyleg
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/drive_control.hpp"
#include "vehicle_abstraction_layer_interfaces/msg/vehicle_state.hpp"
#include "environment_recognition_interfaces/msg/target_vehicle.hpp"
#include "sony_dualshock_three_controller_interfaces/msg/sony_dual_shock_three_controller_input_percentage.hpp"

#include <memory>
#include <optional>
#include <utility>
#include <chrono>
#include <ratio>

using std::placeholders::_1;

using namespace std::chrono_literals;

class Acc : public rclcpp::Node
{
public:
	Acc() :
		Node("drive_strategy_acc")
	{
		this->declare_parameter<double>("target_distance", 10.0);
		this->declare_parameter<double>("target_velocity", 5.0);
		this->declare_parameter<double>("gap_closing_time", 1.0);


		mRemoteControlSubscription = this->create_subscription<sony_dualshock_three_controller_interfaces::msg::SonyDualShockThreeControllerInputPercentage>(
				"/primary_vehicle/vehicle_remote_control/input/SonyDualShockThreeControllerInputPercentage", 10, std::bind(&Acc::remoteControlCallback, this, _1));

		mVehicleStateSubscription = this->create_subscription<vehicle_abstraction_layer_interfaces::msg::VehicleState>(
				"/vehicle_abstraction_layer/output/vehicle_state", 10, std::bind(&Acc::vehicleStateCallback, this, _1));

		mTargetVehicleSubscription = this->create_subscription<environment_recognition_interfaces::msg::TargetVehicle>(
				"/environment_recognition/output/target_vehicle", 10, std::bind(&Acc::targetVehicleCallback, this, _1));

		mDriveControlPublisher = this->create_publisher<vehicle_abstraction_layer_interfaces::msg::DriveControl>(
				"/vehicle_abstraction_layer/input/drive_control", 10);

		mTimer = this->create_wall_timer(100ms, std::bind(&Acc::timerCallback, this));
	}

private:

	void publishDriveControl(double targetVelocity, double targetYawRate) {
		auto driveControl = vehicle_abstraction_layer_interfaces::msg::DriveControl();
		driveControl.set__target_velocity(targetVelocity);
		driveControl.set__target_yaw_rate(targetYawRate);
		mDriveControlPublisher->publish(driveControl);
	}

	void determineAccState() {
		if (!mAccEnabled && mRemoteControlInput->button_cross > 0.25) {
			mAccEnabled = true;
			RCLCPP_INFO(this->get_logger(), "Acc state: enabled");
		} else if (mAccEnabled && mRemoteControlInput->button_circle > 0.25) {
			mAccEnabled = false;
			RCLCPP_INFO(this->get_logger(), "Acc state: disabled");
		}
	}

	double calculateTargetYawRateFromRemoteControl() {
		return 0.6458 * -mRemoteControlInput->analog_stick_left.x;
	}

	double calculateTargetVelocityFromRemoteControl() {
		return  20.0 * -mRemoteControlInput->analog_stick_right.y;
	}

	void timerCallback() {

		const double targetDistance = this->get_parameter("target_distance").as_double();
		const double gapClosingTime = this->get_parameter("gap_closing_time").as_double();
		const double targetYawRate = calculateTargetYawRateFromRemoteControl();

		double targetVelocity;

		if (mAccEnabled) {
			if (mTargetVehicle->available) {
				const float dDistance = mTargetVehicle->distance - targetDistance;
				const float velocityOffset = (dDistance / gapClosingTime);
				targetVelocity = fmax(mTargetVehicle->absolute_velocity + velocityOffset, 0);
			} else {
				targetVelocity = this->get_parameter("target_velocity").as_double();
			}
		} else {
			targetVelocity = calculateTargetVelocityFromRemoteControl();
		}

		publishDriveControl(targetVelocity, targetYawRate);

	}

	void remoteControlCallback(const sony_dualshock_three_controller_interfaces::msg::SonyDualShockThreeControllerInputPercentage::SharedPtr msg) {
		mRemoteControlInput = *msg;
		determineAccState();
	}

	void vehicleStateCallback(const vehicle_abstraction_layer_interfaces::msg::VehicleState::SharedPtr msg) {
		mVehicleState = *msg;
	}

	void targetVehicleCallback(const environment_recognition_interfaces::msg::TargetVehicle::SharedPtr msg) {
		mTargetVehicle = *msg;
	}

	rclcpp::Subscription<sony_dualshock_three_controller_interfaces::msg::SonyDualShockThreeControllerInputPercentage>::SharedPtr mRemoteControlSubscription;
	rclcpp::Subscription<vehicle_abstraction_layer_interfaces::msg::VehicleState>::SharedPtr mVehicleStateSubscription;
	rclcpp::Subscription<environment_recognition_interfaces::msg::TargetVehicle>::SharedPtr mTargetVehicleSubscription;

	rclcpp::Publisher<vehicle_abstraction_layer_interfaces::msg::DriveControl>::SharedPtr mDriveControlPublisher;

	rclcpp::TimerBase::SharedPtr mTimer;


	bool mAccEnabled = false;
	std::optional<vehicle_abstraction_layer_interfaces::msg::VehicleState> mVehicleState;
	std::optional<sony_dualshock_three_controller_interfaces::msg::SonyDualShockThreeControllerInputPercentage> mRemoteControlInput;
	std::optional<environment_recognition_interfaces::msg::TargetVehicle> mTargetVehicle;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Acc>());
	rclcpp::shutdown();
	return 0;
}
