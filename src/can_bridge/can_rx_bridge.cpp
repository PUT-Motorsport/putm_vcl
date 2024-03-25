#include <cstdio>
#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_headers/PM09-CANBUS-FRONTBOX.hpp"
#include "putm_pm09_vcl/msg/detail/frontbox__struct.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "putm_pm09_vcl/msg/frontbox.hpp"

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"

using namespace std::chrono_literals;
using namespace PUTM_CAN;

// class CanRxNode : public rclcpp::Node
// {
//   public:
//     CanRxNode() : Node("can_rx_node"), can_rx("can1", NO_TIMEOUT)
//     {
// 	    CanRxNodeTimer = this->create_wall_timer(1ms, std::bind(&CanRxNode::CanRxNodeMainLoop, this));
//       FrontBoxPublisher = this->create_publisher<putm_pm09_vcl::msg::Frontbox>  ("frontbox",   10);
//     }
//   private:
//     void CanRxNodeMainLoop()
//     {
//     	can_frame frame = can_rx.receive();
//     	switch(frame.can_id)
//     	{
//       	case FRONTBOX_MAIN_CAN_ID:
//       	{
//           auto can_frontbox = PUTM_CAN::convert<PUTM_CAN::Frontbox_main>(frame);
//           putm_pm09_vcl::msg::Frontbox frontbox;
//           frontbox.pedal_position = (((can_frontbox.pedal_position) / 500.0)*100.0);
//           FrontBoxPublisher->publish(frontbox);
//       	}
//     	}
//     }
//     rclcpp::Publisher<putm_pm09_vcl::msg::Frontbox>::SharedPtr FrontBoxPublisher;
//     CanRx can_rx;
//     rclcpp::TimerBase::SharedPtr CanRxNodeTimer;
// };

int main(int argc, char ** argv) 
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("can_rx_bridge");
    rclcpp::Publisher<putm_pm09_vcl::msg::Frontbox>::SharedPtr FrontBoxPublisher = node->create_publisher<putm_pm09_vcl::msg::Frontbox>("frontbox", 10);
      
    CanRx can_rx("can1", NO_TIMEOUT);
    
    while(true)
    {
      can_frame frame = can_rx.receive();
      switch(frame.can_id)
      {
        case FRONTBOX_MAIN_CAN_ID:
      	{
          auto can_frontbox = PUTM_CAN::convert<PUTM_CAN::Frontbox_main>(frame);
          putm_pm09_vcl::msg::Frontbox frontbox;
          frontbox.pedal_position = ((can_frontbox.pedal_position) / 500.0);
          FrontBoxPublisher->publish(frontbox);
      	}
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
