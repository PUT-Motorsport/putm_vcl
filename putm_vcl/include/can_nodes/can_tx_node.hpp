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

  rclcpp::TimerBase::SharedPtr can_tx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_tx_common_timer;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_left_setpoints_subscription;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_right_setpoints_subscription;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_left_setpoints_subscription;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_right_setpoints_subscription;
  
  void can_tx_common_callback();

  void amk_front_left_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  void amk_front_right_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  void amk_rear_left_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  void amk_rear_right_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
};
