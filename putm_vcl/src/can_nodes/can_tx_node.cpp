#include "can_nodes/can_tx_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

CanTxNode::CanTxNode()
    // clang-format off
    : Node("can_tx_node"),
      can_tx_amk(can_interface_amk),
      can_tx_common(can_interface_common),
      can_tx_common_timer(this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this))),
      amk_front_left_setpoints_subscription(
          this->create_subscription<msg::AmkSetpoints>("putm_vcl/amk/front/left/setpoints", 1, 
                                                        std::bind(&CanTxNode::amk_front_left_setpoints_callback, this, _1))),
      amk_front_right_setpoints_subscription(
          this->create_subscription<msg::AmkSetpoints>("putm_vcl/amk/front/right/setpoints", 1,
                                                        std::bind(&CanTxNode::amk_front_right_setpoints_callback, this, _1))),
      amk_rear_left_setpoints_subscription(
          this->create_subscription<msg::AmkSetpoints>("putm_vcl/amk/rear/left/setpoints", 1, 
                                                        std::bind(&CanTxNode::amk_rear_left_setpoints_callback, this, _1))),
      amk_rear_right_setpoints_subscription(
          this->create_subscription<msg::AmkSetpoints>("putm_vcl/amk/rear/right/setpoints", 1,
                                                        std::bind(&CanTxNode::amk_rear_right_setpoints_callback, this, _1))) {}
    // clang-format on      

void CanTxNode::amk_front_left_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg) {
  AmkFrontLeftSetpoints amk_front_left_setpoints;
  amk_front_left_setpoints.amk_control.inverter_on = msg.amk_control.inverter_on;
  amk_front_left_setpoints.amk_control.dc_on = msg.amk_control.dc_on;
  amk_front_left_setpoints.amk_control.enable = msg.amk_control.enable;
  amk_front_left_setpoints.amk_control.error_reset = msg.amk_control.error_reset;
  amk_front_left_setpoints.target_torque = msg.target_torque;
  amk_front_left_setpoints.torque_positive_limit = msg.torque_positive_limit;
  amk_front_left_setpoints.torque_negative_limit = msg.torque_negative_limit;
  can_tx_amk.transmit(amk_front_left_setpoints);
}

void CanTxNode::amk_front_right_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg) {
  AmkFrontRightSetpoints amk_front_right_setpoints;
  amk_front_right_setpoints.amk_control.inverter_on = msg.amk_control.inverter_on;
  amk_front_right_setpoints.amk_control.dc_on = msg.amk_control.dc_on;
  amk_front_right_setpoints.amk_control.enable = msg.amk_control.enable;
  amk_front_right_setpoints.amk_control.error_reset = msg.amk_control.error_reset;
  amk_front_right_setpoints.target_torque = msg.target_torque;
  amk_front_right_setpoints.torque_positive_limit = msg.torque_positive_limit;
  amk_front_right_setpoints.torque_negative_limit = msg.torque_negative_limit;
  can_tx_amk.transmit(amk_front_right_setpoints);
}

void CanTxNode::amk_rear_left_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg) {
  AmkRearLeftSetpoints amk_rear_left_setpoints;
  amk_rear_left_setpoints.amk_control.inverter_on = msg.amk_control.inverter_on;
  amk_rear_left_setpoints.amk_control.dc_on = msg.amk_control.dc_on;
  amk_rear_left_setpoints.amk_control.enable = msg.amk_control.enable;
  amk_rear_left_setpoints.amk_control.error_reset = msg.amk_control.error_reset;
  amk_rear_left_setpoints.target_torque = msg.target_torque;
  amk_rear_left_setpoints.torque_positive_limit = msg.torque_positive_limit;
  amk_rear_left_setpoints.torque_negative_limit = msg.torque_negative_limit;
  can_tx_amk.transmit(amk_rear_left_setpoints);
}

void CanTxNode::amk_rear_right_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg) {
  AmkRearRightSetpoints amk_rear_right_setpoints;
  amk_rear_right_setpoints.amk_control.inverter_on = msg.amk_control.inverter_on;
  amk_rear_right_setpoints.amk_control.dc_on = msg.amk_control.dc_on;
  amk_rear_right_setpoints.amk_control.enable = msg.amk_control.enable;
  amk_rear_right_setpoints.amk_control.error_reset = msg.amk_control.error_reset;
  amk_rear_right_setpoints.target_torque = msg.target_torque;
  amk_rear_right_setpoints.torque_positive_limit = msg.torque_positive_limit;
  amk_rear_right_setpoints.torque_negative_limit = msg.torque_negative_limit;
  can_tx_amk.transmit(amk_rear_right_setpoints);
}

void CanTxNode::can_tx_common_callback() {
  can_tx_common.transmit(pc_main_data);
  can_tx_common.transmit(pc_main_data2);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}
