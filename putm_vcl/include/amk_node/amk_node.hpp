#pragma once

#include <putm_vcl_interfaces/msg/amk_actual_values1.hpp>
#include <putm_vcl_interfaces/msg/amk_actual_values2.hpp>
#include <putm_vcl_interfaces/msg/amk_setpoints.hpp>
#include <putm_vcl_interfaces/msg/rtd.hpp>
#include <putm_vcl_interfaces/msg/setpoints.hpp>
#include <rclcpp/rclcpp.hpp>

class AmkNode : public rclcpp::Node {
 public:
  AmkNode();

 private:
  enum class StateMachine { UNDEFINED = -1, IDLING, STARTUP, TORQUE_CONTROL, SWITCH_OFF, ERROR_HANDLER, ERROR_RESET };

  StateMachine state;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_left_setpoints_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_right_setpoints_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_left_setpoints_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_right_setpoints_publisher;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Setpoints>::SharedPtr setpoints_subscriber;

  rclcpp::TimerBase::SharedPtr setpoints_watchdog;
  rclcpp::TimerBase::SharedPtr amk_state_machine_watchdog;
  rclcpp::TimerBase::SharedPtr amk_state_machine_timer;
  rclcpp::TimerBase::SharedPtr amk_setpoints_timer;

  putm_vcl_interfaces::msg::AmkSetpoints amk_front_left_setpoints;
  putm_vcl_interfaces::msg::AmkSetpoints amk_front_right_setpoints;
  putm_vcl_interfaces::msg::AmkSetpoints amk_rear_left_setpoints;
  putm_vcl_interfaces::msg::AmkSetpoints amk_rear_right_setpoints;
  putm_vcl_interfaces::msg::AmkActualValues1 amk_front_left_actual_values1;
  putm_vcl_interfaces::msg::AmkActualValues1 amk_front_right_actual_values1;
  putm_vcl_interfaces::msg::AmkActualValues1 amk_rear_left_actual_values1;
  putm_vcl_interfaces::msg::AmkActualValues1 amk_rear_right_actual_values1;
  putm_vcl_interfaces::msg::Setpoints setpoints;
  putm_vcl_interfaces::msg::Rtd rtd;

  void rtd_callback(const putm_vcl_interfaces::msg::Rtd::SharedPtr msg);
  void setpoints_callback(const putm_vcl_interfaces::msg::Setpoints::SharedPtr msg);
  std::function<void(const putm_vcl_interfaces::msg::AmkActualValues1::SharedPtr)> amk_actual_values1_callback_factory(
      putm_vcl_interfaces::msg::AmkActualValues1& target);

  void setpoints_watchdog_callback();
  void amk_state_machine_watchdog_callback();

  void amk_state_machine_callback();
  void amk_setpoints_callback();
};
