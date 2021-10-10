#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/range.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class SensorGrabber : public rclcpp::Node
{
public:
	SensorGrabber() :
		Node("sensor_grabber")
	{
		mSensorDataPublisher = this->create_publisher<std_msgs::msg::Float64>("sensors/ultrasound", 10);
		mUltrasoundSensorSubscription = this->create_subscription<sensor_msgs::msg::Range>(
				"ultrasound/range", rclcpp::SensorDataQoS(), std::bind(&SensorGrabber::ultrasoundSensorCallback, this, _1));
	}

private:

	void ultrasoundSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) const {
		auto ultrasoundDistance  = std_msgs::msg::Float64();
		ultrasoundDistance.data = msg->range;
		mSensorDataPublisher->publish(ultrasoundDistance);
	}

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mSensorDataPublisher;
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr mUltrasoundSensorSubscription;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorGrabber>());
	rclcpp::shutdown();
	return 0;
}
