#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values2.hpp"
#include "putm_vcl_interfaces/msg/dashboard.hpp"
#include "putm_vcl_interfaces/msg/frontbox_data.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "rclcpp/rclcpp.hpp"

class CanRxNode : public rclcpp::Node {
 public:
  CanRxNode();

 private:
  PUTM_CAN::CanRx can_rx_amk;
  PUTM_CAN::CanRx can_rx_common;

  rclcpp::TimerBase::SharedPtr can_rx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_rx_common_timer;

  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxData>::SharedPtr frontbox_data_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_left_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_right_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_left_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_right_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Dashboard>::SharedPtr dashboard_publisher;

  void can_rx_amk_callback();
  void can_rx_common_callback();
};
