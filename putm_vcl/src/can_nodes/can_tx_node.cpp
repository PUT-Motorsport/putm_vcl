#include "can_nodes/can_tx_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using namespace std::chrono;

using std::placeholders::_1;

CanTxNode::CanTxNode()
    : Node("can_tx_node"),
      can_tx_amk(can_interface_amk),
      can_tx_common(can_interface_common),

      amk_front_left_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
          "amk/front/left/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkFrontLeftSetpoints>, this, _1))),
      amk_front_right_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
          "amk/front/right/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkFrontRightSetpoints>, this, _1))),
      amk_rear_left_setpoints_subscriber(
          this->create_subscription<msg::AmkSetpoints>("amk/rear/left/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkRearLeftSetpoints>, this, _1))),
      amk_rear_right_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
          "amk/rear/right/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkRearRightSetpoints>, this, _1))),

      amk_front_left_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
          "amk/front/left/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkFrontLeftActualValues1>, this, _1))),
      amk_front_right_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
          "amk/front/right/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkFrontRightActualValues1>, this, _1))),
      amk_rear_left_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
          "amk/rear/left/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkRearLeftActualValues1>, this, _1))),
      amk_rear_right_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
          "amk/rear/right/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkRearRightActualValues1>, this, _1))),

      amk_front_left_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
          "amk/front/left/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkFrontLeftActualValues2>, this, _1))),
      amk_front_right_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
          "amk/front/right/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkFrontRightActualValues2>, this, _1))),
      amk_rear_left_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
          "amk/rear/left/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkRearLeftActualValues2>, this, _1))),
      amk_rear_right_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
          "amk/rear/right/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkRearRightActualValues2>, this, _1))),

      can_tx_common_timer(this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this))) {}

template <typename T>
void CanTxNode::amk_setpoints_callback(const msg::AmkSetpoints msg) {
  T amk_setpoints;
  amk_setpoints.amk_control.inverter_on = msg.amk_control.inverter_on;
  amk_setpoints.amk_control.dc_on = msg.amk_control.dc_on;
  amk_setpoints.amk_control.enable = msg.amk_control.enable;
  amk_setpoints.amk_control.error_reset = msg.amk_control.error_reset;
  amk_setpoints.target_torque = msg.target_torque;
  amk_setpoints.torque_positive_limit = msg.torque_positive_limit;
  amk_setpoints.torque_negative_limit = msg.torque_negative_limit;
  try {
    can_tx_amk.transmit(amk_setpoints);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transmit AMK setpoints: %s", e.what());
  }
}

template <typename T>
void CanTxNode::amk_actual_values1_callback(const msg::AmkActualValues1 msg) {
  T amk_actual_values1;
  amk_actual_values1.amk_status.system_ready = msg.amk_status.system_ready;
  amk_actual_values1.amk_status.error = msg.amk_status.error;
  amk_actual_values1.amk_status.warn = msg.amk_status.warn;
  amk_actual_values1.amk_status.quit_dc_on = msg.amk_status.quit_dc_on;
  amk_actual_values1.amk_status.dc_on = msg.amk_status.dc_on;
  amk_actual_values1.amk_status.quit_inverter_on = msg.amk_status.quit_inverter_on;
  amk_actual_values1.amk_status.inverter_on = msg.amk_status.inverter_on;
  amk_actual_values1.amk_status.derating = msg.amk_status.derating;
  amk_actual_values1.actual_velocity = msg.actual_velocity;
  amk_actual_values1.torque_current = msg.torque_current;
  amk_actual_values1.magnetizing_current = msg.magnetizing_current;

  try {
    can_tx_common.transmit(amk_actual_values1);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transmit AMK actual values 1: %s", e.what());
  }
}

template <typename T>
void CanTxNode::amk_actual_values2_callback(const msg::AmkActualValues2 msg) {
  T amk_actual_values2;
  amk_actual_values2.temp_motor = msg.temp_motor;
  amk_actual_values2.temp_inverter = msg.temp_inverter;
  amk_actual_values2.error_info = msg.error_info;
  amk_actual_values2.temp_igbt = msg.temp_igbt;

  try {
    can_tx_common.transmit(amk_actual_values2);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transmit AMK actual values 2: %s", e.what());
  }
}

void CanTxNode::can_tx_common_callback() {
  std::time_t now_time_t = system_clock::to_time_t(system_clock::now());
  std::tm now_tm = *std::localtime(&now_time_t);
  uint32_t seconds_since_midnight = now_tm.tm_hour * 3600 + now_tm.tm_min * 60 + now_tm.tm_sec;

  PcMainData pc_main_data;
  pc_main_data.time = seconds_since_midnight;

  try {
    can_tx_common.transmit(pc_main_data);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transmit common CAN frames: %s", e.what());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}
