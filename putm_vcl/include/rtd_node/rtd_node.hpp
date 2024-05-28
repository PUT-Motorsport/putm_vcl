#pragma once

#include "putm_vcl_interfaces/msg/dash.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "rclcpp/rclcpp.hpp"

class RtdNode : public rclcpp::Node {
 public:
  RtdNode();

 private:
  putm_vcl_interfaces::msg::Rtd rtd;
  putm_vcl_interfaces::msg::FrontboxDriverInput frontbox_driver_input;
  putm_vcl_interfaces::msg::Dash dash;

  rclcpp::TimerBase::SharedPtr rtd_timer;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_publisher;
  rclcpp::Subscription<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Dash>::SharedPtr dash_subscriber;

  void rtd_callback();
  void frontbox_driver_input_callback(const putm_vcl_interfaces::msg::FrontboxDriverInput::SharedPtr msg);
  void dash_callback(const putm_vcl_interfaces::msg::Dash::SharedPtr msg);
};
