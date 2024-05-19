#include "putm_vcl_interfaces/msg/amk_control.hpp"
#include "putm_vcl_interfaces/msg/amk_status.hpp"
#include "putm_vcl_interfaces/msg/detail/rtd__struct.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using namespace std::placeholders;

enum class Inverters { FRONT_LEFT = 0, FRONT_RIGHT = 1, REAR_LEFT = 2, REAR_RIGHT = 3 };

class AmkNode : public rclcpp::Node {
 public:
  AmkNode();

 private:
  enum class StateMachine { UNDEFINED = -1, IDLING, STARTUP, TOURQE_CONTROL, SWITCH_OFF, ERROR_HANDLER, ERROR_RESET };

  void rtd_callback(const msg::Rtd msg);
  void setpoints_callback(const msg::Setpoints msg);
  void amk_status_callback(const msg::AmkStatus::SharedPtr msg);

  void amk_state_machine_callback();
  void amk_control_callback();

  void amk_startup_watchdog_callback();

  msg::AmkControl amk_control;
  msg::AmkStatus amk_status;

  StateMachine state;
  bool rtd_state;
  int32_t tourqe_setpoints[4];

  rclcpp::TimerBase::SharedPtr amk_control_publisher_timer;
  rclcpp::TimerBase::SharedPtr amk_state_machine_timer;
  rclcpp::TimerBase::SharedPtr amk_startup_watchdog;

  rclcpp::Publisher<msg::AmkControl>::SharedPtr publisher_amk_control;

  rclcpp::Subscription<msg::Rtd>::SharedPtr subscription_rtd;
  rclcpp::Subscription<msg::Setpoints>::SharedPtr subscription_setpoints;
  rclcpp::Subscription<msg::AmkStatus>::SharedPtr subscription_amk_status;
};

AmkNode::AmkNode() : Node("amk_main_node"), state(StateMachine::UNDEFINED), rtd_state(false), tourqe_setpoints{0, 0, 0, 0} {
  publisher_amk_control = this->create_publisher<msg::AmkControl>("putm_vcl/amk_control", 1);

  subscription_rtd = this->create_subscription<msg::Rtd>("putm_vcl/rtd", 1, std::bind(&AmkNode::rtd_callback, this, _1));
  subscription_setpoints = this->create_subscription<msg::Setpoints>("putm_vcl/setpoints", 1, std::bind(&AmkNode::setpoints_callback, this, _1));
  subscription_amk_status = this->create_subscription<msg::AmkStatus>("putm_vcl/amk_status", 1, std::bind(&AmkNode::amk_status_callback, this, _1));

  amk_state_machine_timer = this->create_wall_timer(5ms, std::bind(&AmkNode::amk_state_machine_callback, this));
  amk_control_publisher_timer = this->create_wall_timer(2ms, std::bind(&AmkNode::amk_control_callback, this));
}

void AmkNode::amk_status_callback(const msg::AmkStatus::SharedPtr msg) { amk_status = *msg; }

void AmkNode::amk_control_callback() { publisher_amk_control->publish(amk_control); }

void AmkNode::rtd_callback(const msg::Rtd msg) { rtd_state = msg.rtd_state; }

void AmkNode::amk_startup_watchdog_callback() {
  state = StateMachine::UNDEFINED;
  RCLCPP_INFO(this->get_logger(), "Startup watchdog triggered");
  amk_startup_watchdog->cancel();
}

void AmkNode::setpoints_callback(const msg::Setpoints msg) {
  tourqe_setpoints[0] = msg.tourqes[0];
  tourqe_setpoints[1] = msg.tourqes[1];
  tourqe_setpoints[2] = msg.tourqes[2];
  tourqe_setpoints[3] = msg.tourqes[3];
}

void AmkNode::amk_state_machine_callback() {
  /* State Machine */
  switch (state) {
    case StateMachine::UNDEFINED: {
      if (amk_status.amk_status_berror[0] || amk_status.amk_status_berror[1] || amk_status.amk_status_berror[2] || amk_status.amk_status_berror[3]) {
        RCLCPP_INFO(this->get_logger(), "Detected inverter error");
        state = StateMachine::ERROR_RESET;
      }
      if (amk_status.amk_status_bsystem_ready[0] && amk_status.amk_status_bsystem_ready[1] && amk_status.amk_status_bsystem_ready[2] &&
          amk_status.amk_status_bsystem_ready[3]) {
        RCLCPP_INFO(this->get_logger(), "Inverters online, going to idle");
        state = StateMachine::IDLING;
      }
    } break;
    case StateMachine::ERROR_RESET: {
      if (amk_status.amk_status_bsystem_ready[0] && amk_status.amk_status_bsystem_ready[1] && amk_status.amk_status_bsystem_ready[2] &&
          amk_status.amk_status_bsystem_ready[3]) {
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
      if (rtd_state == 1) {
        state = StateMachine::STARTUP;
        amk_startup_watchdog = this->create_wall_timer(5000ms, std::bind(&AmkNode::amk_startup_watchdog_callback, this));
      }
    } break;
    case StateMachine::STARTUP: {
      amk_control.amk_control_bdc_on.fill(true);
      amk_control.amk_tourqe_positive_limit.fill(0);
      amk_control.amk_tourqe_negative_limit.fill(0);
      amk_control.amk_target_tourqe.fill(0);

      if (!amk_status.amk_status_bdc_on[0] && !amk_status.amk_status_bdc_on[1] && !amk_status.amk_status_bdc_on[2] && !amk_status.amk_status_bdc_on[3]) {
        RCLCPP_INFO(this->get_logger(), "Waiting for bdc_on");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }
      amk_control.amk_control_binverter_on.fill(true);
      amk_control.amk_control_benable.fill(true);
      if (!amk_status.amk_status_binverter_on[0] && !amk_status.amk_status_binverter_on[1] && !amk_status.amk_status_binverter_on[2] &&
          !amk_status.amk_status_binverter_on[3]) {
        RCLCPP_INFO(this->get_logger(), "Waiting for inverter enable");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      }
      RCLCPP_INFO(this->get_logger(), "inverters enabled");
      amk_control.amk_control_benable.fill(true);
      if (!(amk_status.amk_status_bquit_inverter_on[0] && amk_status.amk_status_bquit_inverter_on[1] && amk_status.amk_status_bquit_inverter_on[2] &&
            amk_status.amk_status_bquit_inverter_on[3])) {
        RCLCPP_INFO(this->get_logger(), "Waiting for bquit inverter on");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break;
      } else {
        RCLCPP_INFO(this->get_logger(), "Inverters On");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        amk_startup_watchdog->cancel();
        state = StateMachine::TOURQE_CONTROL;
      }
    } break;
    case StateMachine::TOURQE_CONTROL: {
      /* Check some stop conditions*/
      if (!(amk_status.amk_status_binverter_on[0] && amk_status.amk_status_binverter_on[1] && amk_status.amk_status_binverter_on[2] &&
            amk_status.amk_status_binverter_on[3])) {
        state = StateMachine::SWITCH_OFF;
      }
      amk_control.amk_tourqe_positive_limit.fill(2000);
      amk_control.amk_target_tourqe[0] = tourqe_setpoints[0];
      amk_control.amk_target_tourqe[1] = tourqe_setpoints[1];
      amk_control.amk_target_tourqe[2] = tourqe_setpoints[2];
      amk_control.amk_target_tourqe[3] = tourqe_setpoints[3];
      if (rtd_state != 1) {
        state = StateMachine::SWITCH_OFF;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));

    } break;
    case StateMachine::SWITCH_OFF: {
      amk_control.amk_control_binverter_on.fill(false);
      amk_control.amk_control_benable.fill(false);
      amk_control.amk_control_bdc_on.fill(false);
      amk_control.amk_tourqe_positive_limit.fill(0);
      amk_control.amk_tourqe_negative_limit.fill(0);
      amk_control.amk_target_tourqe.fill(0);
      RCLCPP_INFO(this->get_logger(), "Inverters OFF");
      /* Wait until inverter 0 is switched-off.*/
      if (amk_status.amk_status_binverter_on[0]) {
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
