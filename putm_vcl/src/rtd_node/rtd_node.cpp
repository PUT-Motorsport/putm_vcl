#include "rtd_node/rtd_node.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RtdNode::RtdNode() : Node("rtd_node"), rtd_publisher(this->create_publisher<msg::Rtd>("rtd", 1)) {
  this->create_subscription<msg::FrontboxDriverInput>("frontbox_driver_input", 1, std::bind(&RtdNode::frontbox_driver_input_callback, this, _1));
  this->create_subscription<msg::Dashboard>("dashboard", 1, std::bind(&RtdNode::dashboard_callback, this, _1));
  this->create_wall_timer(100ms, std::bind(&RtdNode::rtd_callback, this));
}

void RtdNode::rtd_callback() {
  if ((frontbox_driver_input.brake_pressure_front >= 2200.0 or frontbox_driver_input.brake_pressure_rear >= 2200.0) and dashboard.rtd_button and not rtd.state) {
    RCLCPP_INFO(this->get_logger(), "RTD: on");
    rtd.state = true;
  } else if (rtd.state and dashboard.rtd_button) {
    RCLCPP_INFO(this->get_logger(), "RTD: off");
    rtd.state = false;
  }
  rtd_publisher->publish(rtd);
}

void RtdNode::frontbox_driver_input_callback(const msg::FrontboxDriverInput::SharedPtr msg) { frontbox_driver_input = *msg; }

void RtdNode::dashboard_callback(const msg::Dashboard::SharedPtr msg) { dashboard = *msg; }

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtdNode>());
  rclcpp::shutdown();
}
