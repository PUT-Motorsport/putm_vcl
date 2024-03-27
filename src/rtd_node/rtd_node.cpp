#include "putm_pm09_vcl/msg/rtd.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <string>

#include "putm_pm09_vcl/msg/dash.hpp"
#include "putm_pm09_vcl/msg/detail/dash__struct.hpp"
#include "putm_pm09_vcl/msg/detail/frontbox__struct.hpp"
#include "putm_pm09_vcl/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"
/* #include "putm_pm09_vcl/msg/hv.hpp" */

using namespace std::chrono_literals;
using std::placeholders::_1;

class Rtd : public rclcpp::Node
{
public:
  Rtd()
  : Node("rtd"),
    rtd_state(false),
    brake_pressure_front(0.0),
    brake_pressure_rear(0.0),
    rtd_button_state(0),
    airs_state(false)
  {
    rtdPublisher = this->create_publisher<putm_pm09_vcl::msg::Rtd>("rtd", 10);
    frontboxSubscriber = this->create_subscription<putm_pm09_vcl::msg::Frontbox>(
      "frontbox", 10, std::bind(&Rtd::frontboxCallback, this, _1));
    dashSubscriber = this->create_subscription<putm_pm09_vcl::msg::Dash>(
      "dash", 10, std::bind(&Rtd::dashCallback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&Rtd::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto rtdMsg = putm_pm09_vcl::msg::Rtd();
    /* Entry condition */
    if ((brake_pressure_front >= 100.0) or (brake_pressure_rear >= 100.0)) {
      RCLCPP_INFO(this->get_logger(), "Braking...");
      if ((rtd_button_state) and not(rtd_state)) {
        RCLCPP_INFO(this->get_logger(), "RTD ON");
        rtd_state = true;
        rtdMsg.rtd_state = rtd_state;
        rtdPublisher->publish(rtdMsg);
      }
    } else if (rtd_state & rtd_button_state) {
      RCLCPP_INFO(this->get_logger(), "RTD OFF");
      rtd_state = false;
      rtdMsg.rtd_state = rtd_state;
      rtdPublisher->publish(rtdMsg);
    }
    /* Exit conditions */
    // if ((rtd_state) and (rtd_button_state))
    // {
    //   rtd_state = false;
    //   RCLCPP_INFO(this->get_logger(), "RTD OFF");
    // }
  }
  void frontboxCallback(const putm_pm09_vcl::msg::Frontbox msg)
  {
    brake_pressure_front = msg.brake_pressure_front;
    brake_pressure_rear = msg.brake_pressure_rear;
  }
  void dashCallback(const putm_pm09_vcl::msg::Dash msg) { rtd_button_state = msg.rtd_button_state; }
  /* Car rtd current rtd_state */
  bool rtd_state;
  /* Car data */
  float brake_pressure_front;
  float brake_pressure_rear;
  bool rtd_button_state;
  bool airs_state;
  /* Node objects */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<putm_pm09_vcl::msg::Rtd>::SharedPtr rtdPublisher;
  rclcpp::Subscription<putm_pm09_vcl::msg::Frontbox>::SharedPtr frontboxSubscriber;
  rclcpp::Subscription<putm_pm09_vcl::msg::Dash>::SharedPtr dashSubscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rtd>());
  rclcpp::shutdown();
  return 0;
}
