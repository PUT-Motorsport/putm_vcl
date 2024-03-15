#include <cstdio>
#include "rclcpp/rclcpp.hpp" 
#include "putm_ev_amk_2023/msg/frontbox.hpp"

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"

using namespace std::chrono_literals;
using namespace PUTM_CAN;

class CanRxNode : public rclcpp::Node
{
  public:
    CanRxNode() : Node("can_rx_node"), count_(0), can_rx("can1", NO_TIMEOUT)
    {
	CanRxNodeTimer = this->create_wall_timer(1ms, std::bind(&CanRxNode::CanRxNodeMainLoop, this));
    }
  private:
    void CanRxNodeMainLoop()
    {
    	can_frame frame = can_rx.receive();
    	switch(frame.can_id)
    	{
      	case FRONTBOX_MAIN_CAN_ID:
      	{

      	}
    	}
    }
    CanRx can_rx;
    rclcpp::TimerBase::SharedPtr CanRxNodeTimer;
    size_t count_;
};

int main(int argc, char ** argv) 
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanRxNode>());
    rclcpp::shutdown();
}
