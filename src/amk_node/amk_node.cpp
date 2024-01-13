#include <cstdio>

#include "rclcpp/rclcpp.hpp" 
#include "putm_ev_amk_2023/msg/amk_status.hpp"
#include "putm_ev_amk_2023/msg/amk_control.hpp"
#include "amk_startup.hpp"

using std::placeholders::_1;

enum class Inverters {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

enum class InverterState {
  UNDEFINED = -1,
  OK = 0,
  ERROR = 1,
  /* More specific states*/
  INVERTERS_ON = 2,
  HV_ON = 3,
  RTD = 4
}state{InverterState::UNDEFINED};

class AmkStatusSubscriber : public rclcpp::Node
{
  public:
    AmkStatusSubscriber() : Node("AMK main node")
    {
      subscription_ = this->create_subscription<putm_ev_amk_2023::msg::AmkStatus>("AmkStatus", 10, std::bind(&AmkStatusSubscriber::AmkStatusCallback, this, _1));
    }
  private:
    void AmkStatusCallback(const putm_ev_amk_2023::msg::AmkStatus::SharedPtr msg) const
    {
      // /* 1. Check if any of the inverter have error*/
      // for (uint8_t inverter = 0; inverter < 4; inverter++)
      // {
      //   if (!msg->amk_status_bsystem_ready){
      //     state = InverterState::ERROR;
      //     break;
      //   }
      // }
      // /* 2. Run state machine */
      // state = InverterState::OK;
      // switch(state)
      // {
      //   /* Inverter state undefined*/
      //   case InverterState::UNDEFINED:
      //   {
          
      //   }
      //   break;
      //   /* Inverter state ok*/
      //   case InverterState::OK:
      //   {
      //     /* Wait for rtd */
      //   }
      //   break;
      //   /* Inverter state error*/
      //   case InverterState::ERROR:
      //   {
          

      //   }
      //   break;
      //   /* default case */
      //   default:
      //   {

      //   }
      //   break;
      // }
    
    }
    rclcpp::Subscription<putm_ev_amk_2023::msg::AmkStatus>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {

  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("amk_node");
  rclcpp::Publisher<putm_ev_amk_2023::msg::AmkControl>::SharedPtr AmkControlPublisher = node->create_publisher<putm_ev_amk_2023::msg::AmkControl>("amk_control", 10);

  putm_ev_amk_2023::msg::AmkControl amk_control;
  amk_control.amk_control_bdc_on = {1,1,1,1};

  while(true)
  {
    AmkControlPublisher->publish(amk_control);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  AMK_Startup();

  // 1. Check for errors
  // 2. if no errors, wait for HV voltage on dc link capacitors.
  // 3. Wait for RTD condition, to enable inverters
  // 4. 
  }

