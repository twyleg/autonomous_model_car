#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class EnvironmentRecognition : public rclcpp::Node
{
public:
	EnvironmentRecognition() :
		Node("environment_recognition")
	{
		mDrivePublisher = this->create_publisher<geometry_msgs::msg::Twist>("/primary_vehicle_control/cmd_vel", 10);
		mSensorDataSubscription = this->create_subscription<std_msgs::msg::Float64>(
				"sensors/ultrasound", 10, std::bind(&EnvironmentRecognition::sensorDataCallback, this, _1));
	}

private:
	void sensorDataCallback(const std_msgs::msg::Float64::SharedPtr msg) const {

		const float distance = msg->data;
		auto driveCommand = geometry_msgs::msg::Twist();

		if (distance > 5) {
			driveCommand.linear.set__x(1);
		} else {
			driveCommand.linear.set__x(0);
		}

		mDrivePublisher->publish(driveCommand);
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mDrivePublisher;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mSensorDataSubscription;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EnvironmentRecognition>());
	rclcpp::shutdown();
	return 0;
}
