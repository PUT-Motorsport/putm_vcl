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

  void can_tx_common_callback();

  template <typename T>
  void amk_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
};
