#include "amk_node/amk_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

AmkNode::AmkNode()
    : Node("amk_node"),
      state(StateMachine::UNDEFINED),

      publisher_amk_front_left_setpoints(this->create_publisher<msg::AmkSetpoints>("putm_vcl/amk/front/left/setpoints", 1)),
      publisher_amk_front_right_setpoints(this->create_publisher<msg::AmkSetpoints>("putm_vcl/amk/front/right/setpoints", 1)),
      publisher_amk_rear_left_setpoints(this->create_publisher<msg::AmkSetpoints>("putm_vcl/amk/rear/left/setpoints", 1)),
      publisher_amk_rear_right_setpoints(this->create_publisher<msg::AmkSetpoints>("putm_vcl/amk/rear/right/setpoints", 1)),

      subscription_amk_front_left_actual_values1(this->create_subscription<msg::AmkActualValues1>(
          "putm_vcl/amk/front/left/actual_values1", 1, std::bind(&AmkNode::amk_front_left_actual_values1_callback, this, _1))),
      subscription_amk_front_right_actual_values1(this->create_subscription<msg::AmkActualValues1>(
          "putm_vcl/amk/front/right/actual_values1", 1, std::bind(&AmkNode::amk_front_right_actual_values1_callback, this, _1))),
      subscription_amk_rear_left_actual_values1(this->create_subscription<msg::AmkActualValues1>(
          "putm_vcl/amk/rear/left/actual_values1", 1, std::bind(&AmkNode::amk_rear_left_actual_values1_callback, this, _1))),
      subscription_amk_rear_right_actual_values1(this->create_subscription<msg::AmkActualValues1>(
          "putm_vcl/amk/rear/right/actual_values1", 1, std::bind(&AmkNode::amk_rear_right_actual_values1_callback, this, _1))),

      subscription_rtd(this->create_subscription<msg::Rtd>("putm_vcl/rtd", 1, std::bind(&AmkNode::rtd_callback, this, _1))),
      amk_state_machine_timer(this->create_wall_timer(5ms, std::bind(&AmkNode::amk_state_machine_callback, this))),
      amk_setpoints_timer(this->create_wall_timer(2ms, std::bind(&AmkNode::amk_setpoints_callback, this))),
      setpoints_watchdog(this->create_wall_timer(500ms, std::bind(&AmkNode::setpoints_watchdog_callback, this))),
      amk_state_machine_watchdog(this->create_wall_timer(5000ms, std::bind(&AmkNode::amk_state_machine_watchdog_callback, this))) {
  setpoints_watchdog->cancel();
  amk_state_machine_watchdog->cancel();
}

// Subscriber callbacks
void AmkNode::rtd_callback(const msg::Rtd::SharedPtr msg) { rtd = *msg; }
void AmkNode::setpoints_callback(const msg::Setpoints::SharedPtr msg) {
  setpoints_watchdog->cancel();
  setpoints = *msg;
  setpoints_watchdog->reset();
}
void AmkNode::amk_front_left_actual_values1_callback(const msg::AmkActualValues1::SharedPtr msg) { amk_front_left_actual_values1 = *msg; }
void AmkNode::amk_front_right_actual_values1_callback(const msg::AmkActualValues1::SharedPtr msg) { amk_front_right_actual_values1 = *msg; }
void AmkNode::amk_rear_left_actual_values1_callback(const msg::AmkActualValues1::SharedPtr msg) { amk_rear_left_actual_values1 = *msg; }
void AmkNode::amk_rear_right_actual_values1_callback(const msg::AmkActualValues1::SharedPtr msg) { amk_rear_right_actual_values1 = *msg; }

// Watchdog callbacks
void AmkNode::setpoints_watchdog_callback() {
  RCLCPP_WARN(this->get_logger(), "Setpoints watchdog triggered");
  setpoints.front_left.torque = 0;
  setpoints.front_right.torque = 0;
  setpoints.rear_left.torque = 0;
  setpoints.rear_right.torque = 0;
  setpoints_watchdog->cancel();
}
void AmkNode::amk_state_machine_watchdog_callback() {
  RCLCPP_WARN(this->get_logger(), "State machine watchdog triggered");
  state = StateMachine::UNDEFINED;
  amk_state_machine_watchdog->cancel();
}

// Timer callbacks
void AmkNode::amk_setpoints_callback() {
  publisher_amk_front_left_setpoints->publish(amk_front_left_setpoints);
  publisher_amk_front_right_setpoints->publish(amk_front_right_setpoints);
  publisher_amk_rear_left_setpoints->publish(amk_rear_left_setpoints);
  publisher_amk_rear_right_setpoints->publish(amk_rear_right_setpoints);
}
void AmkNode::amk_state_machine_callback() {
  switch (state) {
    case StateMachine::UNDEFINED: {
      if (amk_front_left_actual_values1.amk_status.error || amk_front_right_actual_values1.amk_status.error || amk_rear_left_actual_values1.amk_status.error ||
          amk_rear_right_actual_values1.amk_status.error) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Detected inverter error");
        state = StateMachine::ERROR_RESET;
      }
      if (amk_front_left_actual_values1.amk_status.system_ready && amk_front_right_actual_values1.amk_status.system_ready &&
          amk_rear_left_actual_values1.amk_status.system_ready && amk_rear_right_actual_values1.amk_status.system_ready) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Inverters online, going to idle");
        state = StateMachine::IDLING;
      }
    } break;
    case StateMachine::ERROR_RESET: {
      if (amk_front_left_actual_values1.amk_status.system_ready && amk_front_right_actual_values1.amk_status.system_ready &&
          amk_rear_left_actual_values1.amk_status.system_ready && amk_rear_left_actual_values1.amk_status.system_ready) {
        amk_front_left_setpoints.amk_control.error_reset = false;
        amk_front_right_setpoints.amk_control.error_reset = false;
        amk_rear_left_setpoints.amk_control.error_reset = false;
        amk_rear_right_setpoints.amk_control.error_reset = false;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error recovered");
        state = StateMachine::IDLING;
      } else {
        amk_front_left_setpoints.amk_control.error_reset = true;
        amk_front_right_setpoints.amk_control.error_reset = true;
        amk_rear_left_setpoints.amk_control.error_reset = true;
        amk_rear_right_setpoints.amk_control.error_reset = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for error reset");
        rclcpp::sleep_for(10ms);
      }
    } break;
    case StateMachine::IDLING: {
      /* Check for RTD*/
      if (rtd.state == true) {
        state = StateMachine::STARTUP;
        amk_state_machine_watchdog->reset();
      }
    } break;
    case StateMachine::STARTUP: {
      amk_front_left_setpoints.amk_control.dc_on = true;
      amk_front_left_setpoints.torque_positive_limit = 0;
      amk_front_left_setpoints.torque_negative_limit = 0;
      amk_front_left_setpoints.target_torque = 0;

      amk_front_right_setpoints.amk_control.dc_on = true;
      amk_front_right_setpoints.torque_positive_limit = 0;
      amk_front_right_setpoints.torque_negative_limit = 0;
      amk_front_right_setpoints.target_torque = 0;

      amk_rear_left_setpoints.amk_control.dc_on = true;
      amk_rear_left_setpoints.torque_positive_limit = 0;
      amk_rear_left_setpoints.torque_negative_limit = 0;
      amk_rear_left_setpoints.target_torque = 0;

      amk_rear_right_setpoints.amk_control.dc_on = true;
      amk_rear_right_setpoints.torque_positive_limit = 0;
      amk_rear_right_setpoints.torque_negative_limit = 0;
      amk_rear_right_setpoints.target_torque = 0;

      if (!amk_front_left_actual_values1.amk_status.dc_on && !amk_front_right_actual_values1.amk_status.dc_on && !amk_rear_left_actual_values1.amk_status.dc_on &&
          !amk_rear_right_actual_values1.amk_status.dc_on) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for bdc_on");
        rclcpp::sleep_for(10ms);
        break;
      }
      amk_front_left_setpoints.amk_control.inverter_on = true;
      amk_front_left_setpoints.amk_control.enable = true;

      amk_front_right_setpoints.amk_control.inverter_on = true;
      amk_front_right_setpoints.amk_control.enable = true;

      amk_rear_left_setpoints.amk_control.inverter_on = true;
      amk_rear_left_setpoints.amk_control.enable = true;

      amk_rear_right_setpoints.amk_control.inverter_on = true;
      amk_rear_right_setpoints.amk_control.enable = true;

      if (!amk_front_left_actual_values1.amk_status.inverter_on && !amk_front_right_actual_values1.amk_status.inverter_on &&
          !amk_rear_left_actual_values1.amk_status.inverter_on && !amk_rear_right_actual_values1.amk_status.inverter_on) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for inverter enable");
        rclcpp::sleep_for(10ms);
        break;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "inverters enabled");

      amk_front_left_setpoints.amk_control.enable = true;
      amk_front_right_setpoints.amk_control.enable = true;
      amk_rear_left_setpoints.amk_control.enable = true;
      amk_rear_right_setpoints.amk_control.enable = true;

      if (!(amk_front_left_actual_values1.amk_status.quit_inverter_on && amk_front_right_actual_values1.amk_status.quit_inverter_on &&
            amk_rear_left_actual_values1.amk_status.quit_inverter_on && amk_rear_right_actual_values1.amk_status.quit_inverter_on)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for bquit inverter on");
        rclcpp::sleep_for(10ms);
        break;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Inverters On");
        rclcpp::sleep_for(10ms);
        amk_state_machine_watchdog->cancel();
        state = StateMachine::TORQUE_CONTROL;
      }
    } break;
    case StateMachine::TORQUE_CONTROL: {
      /* Check some stop conditions*/
      if (!(amk_front_left_actual_values1.amk_status.inverter_on && amk_front_right_actual_values1.amk_status.inverter_on &&
            amk_rear_left_actual_values1.amk_status.inverter_on && amk_rear_right_actual_values1.amk_status.inverter_on)) {
        state = StateMachine::SWITCH_OFF;
      }
      amk_front_left_setpoints.torque_positive_limit = 2000;
      amk_front_left_setpoints.target_torque = setpoints.front_left.torque;

      amk_front_right_setpoints.torque_positive_limit = 2000;
      amk_front_right_setpoints.target_torque = setpoints.front_right.torque;

      amk_rear_left_setpoints.torque_positive_limit = 2000;
      amk_rear_left_setpoints.target_torque = setpoints.rear_left.torque;

      amk_rear_right_setpoints.torque_positive_limit = 2000;
      amk_rear_right_setpoints.target_torque = setpoints.rear_right.torque;

      if (rtd.state == false) {
        state = StateMachine::SWITCH_OFF;
      }
      rclcpp::sleep_for(5ms);

    } break;
    case StateMachine::SWITCH_OFF: {
      amk_front_left_setpoints.amk_control.inverter_on = false;
      amk_front_left_setpoints.amk_control.enable = false;
      amk_front_left_setpoints.amk_control.dc_on = false;
      amk_front_left_setpoints.torque_positive_limit = 0;
      amk_front_left_setpoints.torque_negative_limit = 0;
      amk_front_left_setpoints.target_torque = 0;

      amk_front_right_setpoints.amk_control.inverter_on = false;
      amk_front_right_setpoints.amk_control.enable = false;
      amk_front_right_setpoints.amk_control.dc_on = false;
      amk_front_right_setpoints.torque_positive_limit = 0;
      amk_front_right_setpoints.torque_negative_limit = 0;
      amk_front_right_setpoints.target_torque = 0;

      amk_rear_left_setpoints.amk_control.inverter_on = false;
      amk_rear_left_setpoints.amk_control.enable = false;
      amk_rear_left_setpoints.amk_control.dc_on = false;
      amk_rear_left_setpoints.torque_positive_limit = 0;
      amk_rear_left_setpoints.torque_negative_limit = 0;
      amk_rear_left_setpoints.target_torque = 0;

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Inverters OFF");
      /* Wait until inverter 0 is switched-off.*/
      if (amk_front_left_actual_values1.amk_status.inverter_on || amk_front_right_actual_values1.amk_status.inverter_on ||
          amk_rear_left_actual_values1.amk_status.inverter_on || amk_rear_right_actual_values1.amk_status.inverter_on) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for inverter switch-off");
        rclcpp::sleep_for(5ms);
        break;
      }
      state = StateMachine::IDLING;
    } break;
    case StateMachine::ERROR_HANDLER: {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error handler");
      rclcpp::sleep_for(1000ms);
    } break;
    default:
      break;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmkNode>());
  rclcpp::shutdown();
}
