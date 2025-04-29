#include "amk_node/amk_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

AmkNode::AmkNode()
    : Node("amk_node"),
      state(StateMachine::UNDEFINED),

      amk_front_left_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/front/left/setpoints", 1)),
      amk_front_right_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/front/right/setpoints", 1)),
      amk_rear_left_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/rear/left/setpoints", 1)),
      amk_rear_right_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/rear/right/setpoints", 1)),
      // clang-format off
      amk_front_left_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/front/left/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_front_left_actual_values1))),
      amk_front_right_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/front/right/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_front_right_actual_values1))),
      amk_rear_left_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/rear/left/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_rear_left_actual_values1))),
      amk_rear_right_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/rear/right/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_rear_right_actual_values1))),
      // clang-format on
      rtd_subscriber(this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&AmkNode::rtd_callback, this, _1))),
      setpoints_subscriber(this->create_subscription<msg::Setpoints>("setpoints", 1, std::bind(&AmkNode::setpoints_callback, this, _1))),

      setpoints_watchdog(this->create_wall_timer(500ms, std::bind(&AmkNode::setpoints_watchdog_callback, this))),
      amk_state_machine_watchdog(this->create_wall_timer(5000ms, std::bind(&AmkNode::amk_state_machine_watchdog_callback, this))),
      amk_state_machine_timer(this->create_wall_timer(5ms, std::bind(&AmkNode::amk_state_machine_callback, this))),
      amk_setpoints_timer(this->create_wall_timer(2ms, std::bind(&AmkNode::amk_setpoints_callback, this))) {
  setpoints_watchdog->cancel();
  amk_state_machine_watchdog->cancel();
}

void AmkNode::rtd_callback(const msg::Rtd::SharedPtr msg) { rtd = *msg; }
void AmkNode::setpoints_callback(const msg::Setpoints::SharedPtr msg) {
  setpoints_watchdog->cancel();
  setpoints = *msg;
  setpoints_watchdog->reset();
}
std::function<void(const msg::AmkActualValues1::SharedPtr msg)> AmkNode::amk_actual_values1_callback_factory(msg::AmkActualValues1& target) {
  return [this, &target](const putm_vcl_interfaces::msg::AmkActualValues1::SharedPtr msg) { target = *msg; };
}

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

void AmkNode::amk_setpoints_callback() {
  amk_front_left_setpoints_publisher->publish(amk_front_left_setpoints);
  amk_front_right_setpoints_publisher->publish(amk_front_right_setpoints);
  amk_rear_left_setpoints_publisher->publish(amk_rear_left_setpoints);
  amk_rear_right_setpoints_publisher->publish(amk_rear_right_setpoints);
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
      //check error and reset error
      if (amk_front_left_actual_values1.amk_status.error || amk_front_right_actual_values1.amk_status.error || amk_rear_left_actual_values1.amk_status.error ||
          amk_rear_right_actual_values1.amk_status.error) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "error happeneed ");
        state = StateMachine::ERROR_RESET;
      }
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
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%f",amk_front_left_actual_values1.amk_status.dc_on);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%f",amk_front_right_actual_values1.amk_status.dc_on);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%f",amk_rear_left_actual_values1.amk_status.dc_on);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%f",amk_rear_right_actual_values1.amk_status.dc_on);
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
            // if (!(amk_front_left_actual_values1.amk_status.quit_inverter_on && amk_front_right_actual_values1.amk_status.quit_inverter_on &&
            // amk_rear_left_actual_values1.amk_status.quit_inverter_on && amk_rear_right_actual_values1.amk_status.quit_inverter_on)) {

        state = StateMachine::SWITCH_OFF;
      }
      amk_front_left_setpoints.torque_positive_limit = 2000;
      amk_front_left_setpoints.torque_negative_limit = -2000;
      amk_front_left_setpoints.target_torque = setpoints.front_left.torque;

      amk_front_right_setpoints.torque_positive_limit = 2000;
      amk_front_right_setpoints.torque_negative_limit = -2000;
      amk_front_right_setpoints.target_torque = setpoints.front_right.torque;

      amk_rear_left_setpoints.torque_positive_limit = 2000;
      amk_rear_left_setpoints.torque_negative_limit = -2000;
      amk_rear_left_setpoints.target_torque = setpoints.rear_left.torque;

      amk_rear_right_setpoints.torque_positive_limit = 2000;
      amk_rear_right_setpoints.torque_negative_limit = -2000;
      amk_rear_right_setpoints.target_torque = setpoints.rear_right.torque;

      if (amk_front_left_actual_values1.actual_velocity > 20000 || amk_front_right_actual_values1.actual_velocity > 20000 || amk_rear_left_actual_values1.actual_velocity > 20000 || amk_rear_right_actual_values1.actual_velocity > 20000)
      {
        amk_front_left_setpoints.target_torque = 0;
        amk_front_right_setpoints.target_torque = 0;
        amk_rear_left_setpoints.target_torque = 0;
        amk_rear_right_setpoints.target_torque = 0;
      }

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

      amk_rear_right_setpoints.amk_control.inverter_on = false;
      amk_rear_right_setpoints.amk_control.enable = false;
      amk_rear_right_setpoints.amk_control.dc_on = false;
      amk_rear_right_setpoints.torque_positive_limit = 0;
      amk_rear_right_setpoints.torque_negative_limit = 0;
      amk_rear_right_setpoints.target_torque = 0;
      // here you can put rtd.state = false.
      // rtd.state = false; // 30 april
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
