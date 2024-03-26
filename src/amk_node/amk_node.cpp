#include <cstdio>

#include "putm_pm09_vcl/msg/detail/rtd__struct.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "putm_pm09_vcl/msg/amk_status.hpp"
#include "putm_pm09_vcl/msg/amk_control.hpp"
#include "putm_pm09_vcl/msg/rtd.hpp"
#include "putm_pm09_vcl/msg/setpoints.hpp"

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

putm_pm09_vcl::msg::AmkControl AmkControlMessage;
putm_pm09_vcl::msg::AmkStatus AmkStatusMessage;

class AmkNode : public rclcpp::Node
{
  public:
    AmkNode()
    : Node("amk_main_node"), rtd_state(false), tourqe_setpoints{0,0,0,0}
    {
      subscription_ = this->create_subscription<putm_pm09_vcl::msg::AmkStatus> ("amk_status", 10, std::bind(&AmkNode::AmkStatusCallback, this, std::placeholders::_1));
      rtdSubscription = this->create_subscription<putm_pm09_vcl::msg::Rtd>("rtd", 10, std::bind(&AmkNode::rtdCallback, this, std::placeholders::_1));
      setpointsSubscriber = this->create_subscription<putm_pm09_vcl::msg::Setpoints>("setpoints", 10, std::bind(&AmkNode::setpointsCallback, this, std::placeholders::_1));
      publisher_    = this->create_publisher   <putm_pm09_vcl::msg::AmkControl>("amk_control", 10);
      AmkControlPublisherTimer = this->create_wall_timer(2ms, std::bind(&AmkNode::AmkControlCallback, this));
      AmkMainLoopTimer         = this->create_wall_timer(5ms, std::bind(&AmkNode::AmkMainLoop, this));
    }
  private:
    void AmkStatusCallback(const putm_pm09_vcl::msg::AmkStatus::SharedPtr msg)
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
    void rtdCallback(const putm_pm09_vcl::msg::Rtd msg)
    {
      rtd_state = msg.rtd_state;
    }
    void setpointsCallback(const putm_pm09_vcl::msg::Setpoints msg)
    {
      tourqe_setpoints[0] = msg.tourqes[0];   
      tourqe_setpoints[1] = msg.tourqes[1];
      tourqe_setpoints[2] = msg.tourqes[2];
      tourqe_setpoints[3] = msg.tourqes[3];
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
          if(rtd_state == 1)
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
          AmkControlMessage.amk_target_tourqe[0] = tourqe_setpoints[0];
          AmkControlMessage.amk_target_tourqe[1] = tourqe_setpoints[1];
          AmkControlMessage.amk_target_tourqe[2] = tourqe_setpoints[2];
          AmkControlMessage.amk_target_tourqe[3] = tourqe_setpoints[3];
          if(rtd_state != 1) 
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
    bool rtd_state;
    int32_t tourqe_setpoints[4];
    rclcpp::TimerBase::SharedPtr AmkControlPublisherTimer;
    rclcpp::TimerBase::SharedPtr AmkMainLoopTimer;
    rclcpp::TimerBase::SharedPtr AmkStartupWatchdog;
    rclcpp::Publisher<putm_pm09_vcl::msg::AmkControl>::SharedPtr publisher_;
    rclcpp::Subscription<putm_pm09_vcl::msg::AmkStatus> ::SharedPtr subscription_;
    rclcpp::Subscription<putm_pm09_vcl::msg::Rtd>::SharedPtr rtdSubscription;
    rclcpp::Subscription<putm_pm09_vcl::msg::Setpoints>::SharedPtr setpointsSubscriber;
};

int main(int argc, char ** argv) 
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AmkNode>());
    rclcpp::shutdown();
}

