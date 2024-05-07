#include <cstdio>

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace PUTM_CAN;

class CanTxNode : public rclcpp::Node
{
public:
  CanTxNode() : Node("can_tx_node"), count_(0), can_rx("can1", NO_TIMEOUT)
  {
    CanTxNodeTimer = this->create_wall_timer(1ms, std::bind(&CanTxNode::CanTxNodeMainLoop, this));
  }

private:
  void CanTxNodeMainLoop()
  {
    can_frame frame = can_rx.receive();
    switch (frame.can_id) {
      case FRONTBOX_MAIN_CAN_ID: {
      }
    }
  }
  size_t count_;
  CanRx can_rx;
  rclcpp::TimerBase::SharedPtr CanTxNodeTimer;
};

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}
