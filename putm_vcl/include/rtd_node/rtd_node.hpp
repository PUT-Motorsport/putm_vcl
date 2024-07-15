#pragma once

#include "putm_vcl_interfaces/msg/dashboard.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "rclcpp/rclcpp.hpp"

class RtdNode : public rclcpp::Node {
 public:
  RtdNode();

 private:
  putm_vcl_interfaces::msg::Rtd rtd;
  putm_vcl_interfaces::msg::FrontboxDriverInput frontbox_driver_input;
  putm_vcl_interfaces::msg::Dashboard dashboard;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_publisher;
  rclcpp::Subscription<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscription;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Dashboard>::SharedPtr dashboard_subscription;
  rclcpp::TimerBase::SharedPtr rtd_timer;

  void rtd_callback();
  void frontbox_driver_input_callback(const putm_vcl_interfaces::msg::FrontboxDriverInput::SharedPtr msg);
  void dashboard_callback(const putm_vcl_interfaces::msg::Dashboard::SharedPtr msg);
};
