#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <string>

#include "putm_pm09_vcl/msg/detail/frontbox__struct.hpp"
#include "putm_pm09_vcl/msg/detail/rtd__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_pm09_vcl/msg/frontbox.hpp"
#include "putm_pm09_vcl/msg/rtd.hpp"
/* #include "putm_pm09_vcl/msg/dash.hpp" */
/* #include "putm_pm09_vcl/msg/hv.hpp" */

using namespace std::chrono_literals;
using std::placeholders::_1;

class Rtd : public rclcpp::Node
{
  public:
    Rtd() : Node("rtd"), rtd_state(false) ,brake_pressure_front(0.0), brake_pressure_rear(0.0), rtd_button_state(0), airs_state(false)
    {
      rtdPublisher = this->create_publisher<putm_pm09_vcl::msg::Rtd>("rtd", 10);
      frontboxSubscriber = this->create_subscription<putm_pm09_vcl::msg::Frontbox>("frontbox", 10,std::bind(&Rtd::frontboxCallback, this, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&Rtd::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      /* Entry condition */
      if ((brake_pressure_front >= 100.0) or (brake_pressure_rear >=100.0))
      {
        if (rtd_button_state)
        {
          rtd_state = true;
        }
      }
      /* Exit conditions */
      if ((rtd_state) and (rtd_button_state))
      {
        rtd_state = false;
      }
      auto rtdMsg = putm_pm09_vcl::msg::Rtd();
      rtdMsg.rtd_state = rtd_state;
      rtdPublisher->publish(rtdMsg);
    }
    void frontboxCallback(const putm_pm09_vcl::msg::Frontbox msg)
    {
      brake_pressure_front = msg.brake_pressure_front;
      brake_pressure_rear = msg.brake_pressure_rear;
    }
    // void dashCallback(const putm_pm09_vcl::msg::dash msg)
    // {
        // rtd_button_state = msg.rtd_button_state;
    // }
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rtd>());
  rclcpp::shutdown();
  return 0;
}
