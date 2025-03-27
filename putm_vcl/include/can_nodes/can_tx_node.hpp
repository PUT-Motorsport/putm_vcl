#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values2.hpp"
#include "putm_vcl_interfaces/msg/amk_setpoints.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "putm_vcl_interfaces/msg/lap_timer.hpp"


#include "rclcpp/rclcpp.hpp"

class CanTxNode : public rclcpp::Node {
 public:
  CanTxNode();

 private:
  PUTM_CAN::CanTx can_tx_amk;
  PUTM_CAN::CanTx can_tx_common;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_left_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_right_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_left_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_right_setpoints_subscriber;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_left_actual_values2_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_right_actual_values2_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_left_actual_values2_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_right_actual_values2_subscriber;

  rclcpp::Subscription<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_subscriber;

  rclcpp::TimerBase::SharedPtr can_tx_common_timer;

  rclcpp::Subscription<putm_vcl_interfaces::msg::LapTimer>::SharedPtr lap_timer_subscriber;

  int16_t inverter_current_rl;
  int16_t wheel_speed_fl;

  putm_vcl_interfaces::msg::Rtd rtd;
  putm_vcl_interfaces::msg::AmkActualValues2 frontLeftActualValues2;
  putm_vcl_interfaces::msg::AmkActualValues2 frontRightActualValues2;
  putm_vcl_interfaces::msg::AmkActualValues2 rearLeftActualValues2;
  putm_vcl_interfaces::msg::AmkActualValues2 rearRightActualValues2;


  void rtd_callback(const putm_vcl_interfaces::msg::Rtd msg);
  void lap_timer_callback(const putm_vcl_interfaces::msg::LapTimer msg);

  void can_tx_common_callback();

  template <typename T>
  void amk_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  template <typename T>
  void amk_actual_values1_callback(const putm_vcl_interfaces::msg::AmkActualValues1 msg);
  template <typename T>
  void amk_actual_values2_callback(const putm_vcl_interfaces::msg::AmkActualValues2 msg);
};
