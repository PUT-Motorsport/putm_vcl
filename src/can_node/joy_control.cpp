#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <string>

#include "putm_pm09_vcl/msg/dash.hpp"
#include "putm_pm09_vcl/msg/detail/dash__struct.hpp"
#include "putm_pm09_vcl/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class joyControl : public rclcpp::Node
{
public:
  joyControl() : Node("joy_control")
  {
    frontboxPublisher = this->create_publisher<putm_pm09_vcl::msg::Frontbox>("frontbox", 10);
    dashPublisher = this->create_publisher<putm_pm09_vcl::msg::Dash>("dash", 10);
    joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&joyControl::joyCallback, this, _1));
    /* timer_ = this->create_wall_timer(100ms, std::bind(&Rtd::timer_callback, this)); */
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy msg)
  {
    auto dash_msg = putm_pm09_vcl::msg::Dash();
    auto frontbox_msg = putm_pm09_vcl::msg::Frontbox();

    dash_msg.rtd_button_state = msg.buttons[0];

    frontbox_msg.pedal_position = (1 - msg.axes[5]) * 50.0;
    frontbox_msg.brake_pressure_front = (1 - msg.axes[2]) * 500;
    frontbox_msg.brake_pressure_rear = (1 - msg.axes[2]) * 500;

    frontboxPublisher->publish(frontbox_msg);
    dashPublisher->publish(dash_msg);
  }
  /* Stored Data */
  /* Node objects */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<putm_pm09_vcl::msg::Frontbox>::SharedPtr frontboxPublisher;
  rclcpp::Publisher<putm_pm09_vcl::msg::Dash>::SharedPtr dashPublisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joyControl>());
  rclcpp::shutdown();
  return 0;
}
