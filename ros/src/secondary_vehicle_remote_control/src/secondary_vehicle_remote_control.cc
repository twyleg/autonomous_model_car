#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SecondaryVehicleControl : public rclcpp::Node
{
public:
	SecondaryVehicleControl() :
		Node("secondary_vehicle_remote_control")
	{
		mDrivePublisher = this->create_publisher<geometry_msgs::msg::Twist>("/secondary_vehicle_control/cmd_vel", 10);
		this->declare_parameter<float>("velocity", 0.0);
		mTimer = this->create_wall_timer(
			  100ms, std::bind(&SecondaryVehicleControl::timerCallback, this));
	}

	void timerCallback() {
		float requestedVelocity;
		this->get_parameter("velocity", requestedVelocity);
		RCLCPP_INFO(this->get_logger(), "Velocity: %f", requestedVelocity);

		auto driveCommand = geometry_msgs::msg::Twist();
		driveCommand.linear.set__x(requestedVelocity);
		mDrivePublisher->publish(driveCommand);
	}

private:

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mDrivePublisher;
	rclcpp::TimerBase::SharedPtr mTimer;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SecondaryVehicleControl>());
	rclcpp::shutdown();
	return 0;
}
