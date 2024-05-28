#include "rtd_node/rtd_node.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RtdNode::RtdNode()
    : Node("rtd_node"),
      rtd_timer(this->create_wall_timer(100ms, std::bind(&RtdNode::rtd_callback, this))),
      rtd_publisher(this->create_publisher<msg::Rtd>("putm_vcl/rtd", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<msg::FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&RtdNode::frontbox_driver_input_callback, this, _1))),
      dash_subscriber(this->create_subscription<msg::Dash>("putm_vcl/dash", 1, std::bind(&RtdNode::dash_callback, this, _1))) {}

void RtdNode::rtd_callback() {
  /* Entry condition */
  if ((frontbox_driver_input.brake_pressure_front >= 100.0) or (frontbox_driver_input.brake_pressure_rear >= 100.0)) {
    RCLCPP_INFO(this->get_logger(), "Braking...");
    if ((dash.rtd_button_state) and not(rtd.state)) {
      RCLCPP_INFO(this->get_logger(), "RTD ON");
      rtd.state = true;
      rtd_publisher->publish(rtd);
    }
  } else if (rtd.state & dash.rtd_button_state) {
    RCLCPP_INFO(this->get_logger(), "RTD OFF");
    rtd_publisher->publish(rtd);
  }
  /* Exit conditions */
  // if ((rtd_state) and (rtd_button_state))
  // {
  //   rtd_state = false;
  //   RCLCPP_INFO(this->get_logger(), "RTD OFF");
  // }
}

void RtdNode::frontbox_driver_input_callback(const msg::FrontboxDriverInput::SharedPtr msg) { frontbox_driver_input = *msg; }

void RtdNode::dash_callback(const msg::Dash::SharedPtr msg) { dash = *msg; }

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtdNode>());
  rclcpp::shutdown();
}
