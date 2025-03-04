#include "rtd_node/rtd_node.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RtdNode::RtdNode()
    : Node("rtd_node"),
      rtd_publisher(this->create_publisher<msg::Rtd>("rtd", 1)),
      frontbox_driver_input_subscription(this->create_subscription<msg::FrontboxDriverInput>("frontbox_driver_input", 1, std::bind(&RtdNode::frontbox_driver_input_callback, this, _1))),
      dashboard_subscription(this->create_subscription<msg::Dashboard>("dashboard", 1, std::bind(&RtdNode::dashboard_callback, this, _1))),
      rtd_timer(this->create_wall_timer(100ms, std::bind(&RtdNode::rtd_callback, this))),
      amk_front_left_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/front/left/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_front_left_actual_values1))),
      amk_front_right_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/front/right/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_front_right_actual_values1))),
      amk_rear_left_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/rear/left/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_rear_left_actual_values1))),
      amk_rear_right_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/rear/right/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_rear_right_actual_values1))){}

void RtdNode::rtd_callback() {
  if ((frontbox_driver_input.brake_pressure_front >= 900 )and (frontbox_driver_input.brake_pressure_rear >= 900) and (dashboard.rtd_button) and (not rtd.state)) 
  {
    RCLCPP_INFO(this->get_logger(), "RTD: on");
    rtd.state = true;
  } else if (dashboard.rtd_button and rtd.state)// or (amk_front_left_actual_values1.amk_status.error or amk_front_right_actual_values1.amk_status.error or amk_rear_left_actual_values1.amk_status.error or amk_rear_right_actual_values1.amk_status.error )) 
  { 
    RCLCPP_INFO(this->get_logger(), "RTD: off");
    rtd.state = false;
  }
  rtd_publisher->publish(rtd);
}

void RtdNode::frontbox_driver_input_callback(const msg::FrontboxDriverInput::SharedPtr msg) { frontbox_driver_input = *msg; }

void RtdNode::dashboard_callback(const msg::Dashboard::SharedPtr msg) { dashboard = *msg; }

std::function<void(const msg::AmkActualValues1::SharedPtr msg)> RtdNode::amk_actual_values1_callback_factory(msg::AmkActualValues1& target) {
  return [this, &target](const putm_vcl_interfaces::msg::AmkActualValues1::SharedPtr msg) { target = *msg; };
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtdNode>());
  rclcpp::shutdown();
}
