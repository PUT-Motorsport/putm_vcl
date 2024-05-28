#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"
#include "putm_vcl_interfaces/msg/amk_control.hpp"
#include "rclcpp/rclcpp.hpp"

class CanTxNode : public rclcpp::Node {
 public:
  CanTxNode();

 private:
  PUTM_CAN::CanTx can_tx_amk;
  PUTM_CAN::CanTx can_tx_common;

  PUTM_CAN::AmkFrontLeftSetpoints1 amk_front_left_setpoints;
  PUTM_CAN::AmkFrontRightSetpoints1 amk_front_right_setpoints;
  PUTM_CAN::AmkRearLeftSetpoints1 amk_rear_left_setpoints;
  PUTM_CAN::AmkRearRightSetpoints1 amk_rear_right_setpoints;

  rclcpp::TimerBase::SharedPtr can_tx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_tx_common_timer;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkControl>::SharedPtr amk_control_subscription;

  void can_tx_amk_callback();
  void can_tx_common_callback();

  void amk_control_callback(const putm_vcl_interfaces::msg::AmkControl msg);
};
