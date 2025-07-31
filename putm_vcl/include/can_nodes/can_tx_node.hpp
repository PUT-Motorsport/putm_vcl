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

  int16_t avg_torque_current = 0;
  int16_t torque_current_rr = 0;
  int16_t torque_current_rl = 0;


  int16_t avg_wheel_speed = 0;
  int16_t wheel_speed_rl = 0;
  int16_t wheel_speed_rr = 0;

  bool inverter_ready_rr = 0;
  bool inverter_ready_rl = 0;

  bool inverter_on_rr = 0;
  bool inverter_error_rr = 0;
  bool inverter_on_rl = 0;
  bool inverter_error_rl = 0;

  // bool inverter_on_fr = 0;
  // bool inverter_error_fr = 0;
  // bool inverter_on_fl = 0;
  // bool inverter_error_fl = 0;

  int8_t inverter_temp_rr = 0;
  int8_t inverter_temp_rl = 0;
  int8_t inverter_temp_fr = 0;
  int8_t inverter_temp_fl = 0;

  int8_t motor_temp_rr = 0;
  int8_t motor_temp_rl = 0;
  int8_t motor_temp_fr = 0;
  int8_t motor_temp_fl = 0;

  const uint8_t amk_data_limiter = 90; // amk_data_limiter is a const value that limits how often inverters info is sent on CAN frame. A value "90" means 10ms * 90 = 900ms.
  uint8_t amk_data_limiter_counter = 0; // amk_data_limiter_counter is a variable that counts down from "amkdatalimiter" value to 0, for every call of can_tx_common_callback. When amkdatalimiter reaches "0" only then will the AMKTEMP Frame be sent to CAN1.

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
