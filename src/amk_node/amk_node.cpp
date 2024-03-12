#include <cstdio>

#include "rclcpp/rclcpp.hpp" 
#include "putm_ev_amk_2023/msg/amk_status.hpp"
#include "putm_ev_amk_2023/msg/amk_control.hpp"
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

enum class Inverters {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

enum class StateMachine {
  UNDEFINED = -1,
  IDLING = 0,
  STARTUP = 1,
  TOURQE_CONTROL = 2,
  SWITCH_OFF = 4,
  ERROR_HANDLER = 5,
  ERROR_RESET = 6
}state{StateMachine::UNDEFINED};

putm_ev_amk_2023::msg::AmkControl AmkControlMessage;
putm_ev_amk_2023::msg::AmkStatus AmkStatusMessage;
sensor_msgs::msg::Joy Padinput {};

class AmkNode : public rclcpp::Node
{
  public:
    AmkNode()
    : Node("amk_main_node"), count_(0)
    {
      subscription_ = this->create_subscription<putm_ev_amk_2023::msg::AmkStatus> ("amk_status", 10, std::bind(&AmkNode::AmkStatusCallback, this, std::placeholders::_1));
      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("/joy", 10, std::bind(&AmkNode::joyCallback, this, std::placeholders::_1));
      publisher_    = this->create_publisher   <putm_ev_amk_2023::msg::AmkControl>("amk_control", 10);
      AmkControlPublisherTimer = this->create_wall_timer(2ms, std::bind(&AmkNode::AmkControlCallback, this));
      AmkMainLoopTimer         = this->create_wall_timer(5ms, std::bind(&AmkNode::AmkMainLoop, this));
    }
  private:
    void joyCallback (const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      AmkControlMessage.amk_target_tourqe.fill(-1.0*msg->axes[5]*1000.0 + 1000);
      Padinput = *msg;
    }
    void AmkStatusCallback(const putm_ev_amk_2023::msg::AmkStatus::SharedPtr msg)
    {
      AmkStatusMessage = *msg;
    }
    void AmkControlCallback()
    {
      publisher_->publish(AmkControlMessage);
    }
    void AmkStartupWatchdogCallback()
    {
      state = StateMachine::UNDEFINED;
      RCLCPP_INFO(this->get_logger(), "Startup Watchodg triggered");
      AmkStartupWatchdog->cancel();
    }
    void AmkMainLoop()
    {
      /* State Machine */
      switch(state)
      {
        case StateMachine::UNDEFINED: 
        {
          if (AmkStatusMessage.amk_status_berror[0] || AmkStatusMessage.amk_status_berror[1] || AmkStatusMessage.amk_status_berror[2] || AmkStatusMessage.amk_status_berror[3])
          {
            RCLCPP_INFO(this->get_logger(), "Detected inverter error");
            state = StateMachine::ERROR_RESET;
          }
          if (AmkStatusMessage.amk_status_bsystem_ready[0] && AmkStatusMessage.amk_status_bsystem_ready[1] && AmkStatusMessage.amk_status_bsystem_ready[2] && AmkStatusMessage.amk_status_bsystem_ready[3])
          {
            RCLCPP_INFO(this->get_logger(), "Inverters online, going to idle");
            state = StateMachine::IDLING;
          }
        }
        break;
        case StateMachine::ERROR_RESET: 
        {
          if (AmkStatusMessage.amk_status_bsystem_ready[0] && AmkStatusMessage.amk_status_bsystem_ready[1] && AmkStatusMessage.amk_status_bsystem_ready[2] && AmkStatusMessage.amk_status_bsystem_ready[3])
          {
            AmkControlMessage.amk_control_amkb_error_reset.fill(false);
            RCLCPP_INFO(this->get_logger(), "Error recovered");
            state = StateMachine::IDLING;
          }
          else
          {
            AmkControlMessage.amk_control_amkb_error_reset.fill(true);
            RCLCPP_INFO(this->get_logger(), "Waiting for error reset");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
        break;
        case StateMachine::IDLING: 
        {
          /* Check for RTD*/
          if(Padinput.buttons[0] == 1)
          {
            state = StateMachine::STARTUP;
            AmkStartupWatchdog = this->create_wall_timer(5000ms, std::bind(&AmkNode::AmkStartupWatchdogCallback, this));
          }
        }
        break;
        case StateMachine::STARTUP:
        {
          AmkControlMessage.amk_control_bdc_on.fill(true);
          AmkControlMessage.amk_tourqe_positive_limit.fill(0);
          AmkControlMessage.amk_tourqe_negative_limit.fill(0);
          AmkControlMessage.amk_target_tourqe.fill(0);

          if(!AmkStatusMessage.amk_status_bdc_on[0] && !AmkStatusMessage.amk_status_bdc_on[1] && !AmkStatusMessage.amk_status_bdc_on[2] && !AmkStatusMessage.amk_status_bdc_on[3])
          {
            RCLCPP_INFO(this->get_logger(), "Waiting for bdc_on");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            break;
          }
          AmkControlMessage.amk_control_binverter_on.fill(true);
          AmkControlMessage.amk_control_benable.fill(true);
          if(!AmkStatusMessage.amk_status_binverter_on[0] && !AmkStatusMessage.amk_status_binverter_on[1] && !AmkStatusMessage.amk_status_binverter_on[2] && !AmkStatusMessage.amk_status_binverter_on[3])
          {
            RCLCPP_INFO(this->get_logger(), "Waiting for inverter enable");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            break;
          }
          RCLCPP_INFO(this->get_logger(), "inverters enabled");
          AmkControlMessage.amk_control_benable.fill(true);
          if(!(AmkStatusMessage.amk_status_bquit_inverter_on[0] && AmkStatusMessage.amk_status_bquit_inverter_on[1] && AmkStatusMessage.amk_status_bquit_inverter_on[2] && AmkStatusMessage.amk_status_bquit_inverter_on[3]))
          {
            RCLCPP_INFO(this->get_logger(), "Waiting for bquit inverter on");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            break;
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Inverters On");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            AmkStartupWatchdog->cancel();
            state = StateMachine::TOURQE_CONTROL;
          }
        }
        break;
        case StateMachine::TOURQE_CONTROL:
        {
          /* Check some stop conditions*/
          if(!(AmkStatusMessage.amk_status_binverter_on[0] && AmkStatusMessage.amk_status_binverter_on[1] && AmkStatusMessage.amk_status_binverter_on[2] && AmkStatusMessage.amk_status_binverter_on[3])) {state = StateMachine::SWITCH_OFF;}
          AmkControlMessage.amk_tourqe_positive_limit.fill(2000);

          if(Padinput.buttons[0] == 1) 
          { 
            state = StateMachine::SWITCH_OFF;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));

        }
        break;
        case StateMachine::SWITCH_OFF:
        {
          AmkControlMessage.amk_control_binverter_on.fill(false);
          AmkControlMessage.amk_control_benable.fill(false);
          AmkControlMessage.amk_control_bdc_on.fill(false);
          AmkControlMessage.amk_tourqe_positive_limit.fill(0);
          AmkControlMessage.amk_tourqe_negative_limit.fill(0);
          AmkControlMessage.amk_target_tourqe.fill(0);
          RCLCPP_INFO(this->get_logger(), "Inverters OFF");
          /* Wait until inverter 0 is switched-off.*/
          if (AmkStatusMessage.amk_status_binverter_on[0])
          {
            RCLCPP_INFO(this->get_logger(), "Waiting for inverter switch-off");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            break;
          }
          state = StateMachine::IDLING;
        }
        break;
        case StateMachine::ERROR_HANDLER:
        {
          RCLCPP_INFO(this->get_logger(), "Error handler");
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        default: 
        {

        }
        break;
      }
    }
    rclcpp::TimerBase::SharedPtr AmkControlPublisherTimer;
    rclcpp::TimerBase::SharedPtr AmkMainLoopTimer;
    rclcpp::TimerBase::SharedPtr AmkStartupWatchdog;
    rclcpp::Publisher<putm_ev_amk_2023::msg::AmkControl>::SharedPtr publisher_;
    rclcpp::Subscription<putm_ev_amk_2023::msg::AmkStatus> ::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy> ::SharedPtr joy_subscription_;
    size_t count_;
};

int main(int argc, char ** argv) 
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AmkNode>());
    rclcpp::shutdown();
}

