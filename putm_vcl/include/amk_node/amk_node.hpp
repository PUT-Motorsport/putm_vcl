#pragma once

#include <putm_vcl_interfaces/msg/amk_setpoints.hpp>
#include <putm_vcl_interfaces/msg/amk_status.hpp>
#include <putm_vcl_interfaces/msg/rtd.hpp>
#include <putm_vcl_interfaces/msg/setpoints.hpp>
#include <rclcpp/rclcpp.hpp>

class AmkNode : public rclcpp::Node {
 public:
  AmkNode();

 private:
  enum class StateMachine { UNDEFINED = -1, IDLING, STARTUP, TORQUE_CONTROL, SWITCH_OFF, ERROR_HANDLER, ERROR_RESET };

  StateMachine state;

  putm_vcl_interfaces::msg::AmkSetpoints amk_front_left_setpoints;
  putm_vcl_interfaces::msg::AmkSetpoints amk_front_right_setpoints;
  putm_vcl_interfaces::msg::AmkSetpoints amk_rear_left_setpoints;
  putm_vcl_interfaces::msg::AmkSetpoints amk_rear_right_setpoints;

  putm_vcl_interfaces::msg::AmkStatus amk_status;
  putm_vcl_interfaces::msg::Setpoints setpoints;
  putm_vcl_interfaces::msg::Rtd rtd;

  // Publishers
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr publisher_amk_front_left_setpoints;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr publisher_amk_front_right_setpoints;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr publisher_amk_rear_left_setpoints;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr publisher_amk_rear_right_setpoints;
  // Subscribers
  rclcpp::Subscription<putm_vcl_interfaces::msg::Rtd>::SharedPtr subscription_rtd;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Setpoints>::SharedPtr subscription_setpoints;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkStatus>::SharedPtr subscription_amk_status;
  // Timers
  rclcpp::TimerBase::SharedPtr amk_state_machine_timer;
  rclcpp::TimerBase::SharedPtr amk_setpoints_timer;
  // Watchdogs
  rclcpp::TimerBase::SharedPtr setpoints_watchdog;
  rclcpp::TimerBase::SharedPtr amk_state_machine_watchdog;

  // Subscriber callbacks
  void rtd_callback(const putm_vcl_interfaces::msg::Rtd::SharedPtr msg);
  void setpoints_callback(const putm_vcl_interfaces::msg::Setpoints::SharedPtr msg);
  void amk_status_callback(const putm_vcl_interfaces::msg::AmkStatus::SharedPtr msg);
  // Watchdog callbacks
  void setpoints_watchdog_callback();
  void amk_state_machine_watchdog_callback();
  // Timer callbacks
  void amk_setpoints_callback();
  void amk_state_machine_callback();
};
