#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"
#include "putm_vcl_interfaces/msg/amk_setpoints.hpp"
#include "rclcpp/rclcpp.hpp"

class CanTxNode : public rclcpp::Node {
 public:
  CanTxNode();

 private:
  PUTM_CAN::CanTx can_tx_amk;
  PUTM_CAN::CanTx can_tx_common;

  PUTM_CAN::PcMainData pc_main_data;
  PUTM_CAN::PcMainData pc_main_data2;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_left_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_right_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_left_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_right_setpoints_subscriber;

  rclcpp::TimerBase::SharedPtr can_tx_common_timer;

  void can_tx_common_callback();

  template <typename T>
  void amk_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
};
