#pragma once

#include "putm_vcl_interfaces/msg/dash.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "rclcpp/rclcpp.hpp"

class RtdNode : public rclcpp::Node {
 public:
  RtdNode();

 private:
  putm_vcl_interfaces::msg::Rtd rtd;
  putm_vcl_interfaces::msg::Frontbox frontbox;
  putm_vcl_interfaces::msg::Dash dash;

  rclcpp::TimerBase::SharedPtr rtd_timer;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_publisher;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Frontbox>::SharedPtr frontbox_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Dash>::SharedPtr dash_subscriber;

  void rtd_callback();
  void frontbox_callback(const putm_vcl_interfaces::msg::Frontbox::SharedPtr msg);
  void dash_callback(const putm_vcl_interfaces::msg::Dash::SharedPtr msg);
};
