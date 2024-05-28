#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/amk_data.hpp"
#include "putm_vcl_interfaces/msg/amk_status.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"

class CanRxNode : public rclcpp::Node {
 public:
  CanRxNode();

 private:
  enum Inverters { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };

  PUTM_CAN::CanRx can_rx_amk;
  PUTM_CAN::CanRx can_rx_common;

  putm_vcl_interfaces::msg::AmkStatus amk_status;
  putm_vcl_interfaces::msg::AmkData amk_data;

  rclcpp::TimerBase::SharedPtr can_rx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_rx_common_timer;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Frontbox>::SharedPtr frontbox_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkStatus>::SharedPtr amk_status_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkData>::SharedPtr amk_data_publisher;

  void can_rx_amk_callback();
  void can_rx_common_callback();
};
