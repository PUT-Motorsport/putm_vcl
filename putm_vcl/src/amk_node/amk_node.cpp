#include "putm_vcl_interfaces/msg/amk_control.hpp"
#include "putm_vcl_interfaces/msg/amk_status.hpp"
#include "putm_vcl_interfaces/msg/detail/rtd__struct.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using namespace std::placeholders;

class AmkNode : public rclcpp::Node {
 public:
  AmkNode();

 private:
  enum Inverters { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };
  enum class StateMachine { UNDEFINED = -1, IDLING, STARTUP, TORQUE_CONTROL, SWITCH_OFF, ERROR_HANDLER, ERROR_RESET };

  void rtd_callback(const msg::Rtd::SharedPtr msg);
  void setpoints_callback(const msg::Setpoints::SharedPtr msg);
  void amk_status_callback(const msg::AmkStatus::SharedPtr msg);

  void amk_state_machine_callback();
  void amk_control_callback();

  void amk_startup_watchdog_callback();

  StateMachine state;

  msg::AmkControl amk_control;
  msg::AmkStatus amk_status;
  msg::Setpoints setpoints;
  msg::Rtd rtd;

  rclcpp::TimerBase::SharedPtr amk_control_publisher_timer;
  rclcpp::TimerBase::SharedPtr amk_state_machine_timer;
  rclcpp::TimerBase::SharedPtr amk_startup_watchdog;

  rclcpp::Publisher<msg::AmkControl>::SharedPtr publisher_amk_control;

  rclcpp::Subscription<msg::Rtd>::SharedPtr subscription_rtd;
  rclcpp::Subscription<msg::Setpoints>::SharedPtr subscription_setpoints;
  rclcpp::Subscription<msg::AmkStatus>::SharedPtr subscription_amk_status;
};

AmkNode::AmkNode() : Node("amk_main_node"), state(StateMachine::UNDEFINED) {
  publisher_amk_control = this->create_publisher<msg::AmkControl>("putm_vcl/amk_control", 1);

  subscription_rtd = this->create_subscription<msg::Rtd>("putm_vcl/rtd", 1, std::bind(&AmkNode::rtd_callback, this, _1));
  subscription_setpoints = this->create_subscription<msg::Setpoints>("putm_vcl/setpoints", 1, std::bind(&AmkNode::setpoints_callback, this, _1));
  subscription_amk_status = this->create_subscription<msg::AmkStatus>("putm_vcl/amk_status", 1, std::bind(&AmkNode::amk_status_callback, this, _1));

  amk_state_machine_timer = this->create_wall_timer(5ms, std::bind(&AmkNode::amk_state_machine_callback, this));
  amk_control_publisher_timer = this->create_wall_timer(2ms, std::bind(&AmkNode::amk_control_callback, this));
}

void AmkNode::rtd_callback(const msg::Rtd::SharedPtr msg) { rtd = *msg; }
void AmkNode::setpoints_callback(const msg::Setpoints::SharedPtr msg) { setpoints = *msg; }
void AmkNode::amk_status_callback(const msg::AmkStatus::SharedPtr msg) { amk_status = *msg; }

void AmkNode::amk_control_callback() { publisher_amk_control->publish(amk_control); }

void AmkNode::amk_startup_watchdog_callback() {
  state = StateMachine::UNDEFINED;
  RCLCPP_INFO(this->get_logger(), "Startup watchdog triggered");
  amk_startup_watchdog->cancel();
}

void AmkNode::amk_state_machine_callback() {
  switch (state) {
    case StateMachine::UNDEFINED: {
      if (amk_status.amk_status_berror[Inverters::FRONT_LEFT] || amk_status.amk_status_berror[Inverters::FRONT_RIGHT] ||
          amk_status.amk_status_berror[Inverters::REAR_LEFT] || amk_status.amk_status_berror[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO(this->get_logger(), "Detected inverter error");
        state = StateMachine::ERROR_RESET;
      }
      if (amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] &&
          amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO(this->get_logger(), "Inverters online, going to idle");
        state = StateMachine::IDLING;
      }
    } break;
    case StateMachine::ERROR_RESET: {
      if (amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] &&
          amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT]) {
        amk_control.amk_control_amkb_error_reset.fill(false);
        RCLCPP_INFO(this->get_logger(), "Error recovered");
        state = StateMachine::IDLING;
      } else {
        amk_control.amk_control_amkb_error_reset.fill(true);
        RCLCPP_INFO(this->get_logger(), "Waiting for error reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } break;
    case StateMachine::IDLING: {
      /* Check for RTD*/
      if (rtd.rtd_state == true) {
        state = StateMachine::STARTUP;
        amk_startup_watchdog = this->create_wall_timer(5000ms, std::bind(&AmkNode::amk_startup_watchdog_callback, this));
      }
    } break;
    case StateMachine::STARTUP: {
      amk_control.amk_control_bdc_on.fill(true);
      amk_control.amk_torque_positive_limit.fill(0);
      amk_control.amk_torque_negative_limit.fill(0);
      amk_control.amk_target_torque.fill(0);

      if (!amk_status.amk_status_bdc_on[Inverters::FRONT_LEFT] && !amk_status.amk_status_bdc_on[Inverters::FRONT_RIGHT] &&
          !amk_status.amk_status_bdc_on[Inverters::REAR_LEFT] && !amk_status.amk_status_bdc_on[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO(this->get_logger(), "Waiting for bdc_on");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }
      amk_control.amk_control_binverter_on.fill(true);
      amk_control.amk_control_benable.fill(true);
      if (!amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] && !amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] &&
          !amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] && !amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO(this->get_logger(), "Waiting for inverter enable");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }
      RCLCPP_INFO(this->get_logger(), "inverters enabled");
      amk_control.amk_control_benable.fill(true);
      if (!(amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_LEFT] && amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_RIGHT] &&
            amk_status.amk_status_bquit_inverter_on[Inverters::REAR_LEFT] && amk_status.amk_status_bquit_inverter_on[Inverters::REAR_RIGHT])) {
        RCLCPP_INFO(this->get_logger(), "Waiting for bquit inverter on");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      } else {
        RCLCPP_INFO(this->get_logger(), "Inverters On");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        amk_startup_watchdog->cancel();
        state = StateMachine::TORQUE_CONTROL;
      }
    } break;
    case StateMachine::TORQUE_CONTROL: {
      /* Check some stop conditions*/
      if (!(amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] && amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] &&
            amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] && amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT])) {
        state = StateMachine::SWITCH_OFF;
      }
      amk_control.amk_torque_positive_limit.fill(2000);
      amk_control.amk_target_torque[Inverters::FRONT_LEFT] = setpoints.torques[Inverters::FRONT_LEFT];
      amk_control.amk_target_torque[Inverters::FRONT_RIGHT] = setpoints.torques[Inverters::FRONT_RIGHT];
      amk_control.amk_target_torque[Inverters::REAR_LEFT] = setpoints.torques[Inverters::REAR_LEFT];
      amk_control.amk_target_torque[Inverters::REAR_RIGHT] = setpoints.torques[Inverters::REAR_RIGHT];
      if (rtd.rtd_state == false) {
        state = StateMachine::SWITCH_OFF;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));

    } break;
    case StateMachine::SWITCH_OFF: {
      amk_control.amk_control_binverter_on.fill(false);
      amk_control.amk_control_benable.fill(false);
      amk_control.amk_control_bdc_on.fill(false);
      amk_control.amk_torque_positive_limit.fill(0);
      amk_control.amk_torque_negative_limit.fill(0);
      amk_control.amk_target_torque.fill(0);
      RCLCPP_INFO(this->get_logger(), "Inverters OFF");
      /* Wait until inverter 0 is switched-off.*/
      if (amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT]) {
        RCLCPP_INFO(this->get_logger(), "Waiting for inverter switch-off");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        break;
      }
      state = StateMachine::IDLING;
    } break;
    case StateMachine::ERROR_HANDLER: {
      RCLCPP_INFO(this->get_logger(), "Error handler");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    default:
      break;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmkNode>());
  rclcpp::shutdown();
}
