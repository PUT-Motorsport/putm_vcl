#include <cstdio>

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace PUTM_CAN;

class CanTxNode : public rclcpp::Node {
 public:
  CanTxNode();

 private:
  CanRx can_rx;
  rclcpp::TimerBase::SharedPtr can_tx_node_timer;

  void can_tx_main_node_loop();
};

CanTxNode::CanTxNode() : Node("can_tx_node"), can_rx("can1", NO_TIMEOUT) {
  can_tx_node_timer = this->create_wall_timer(1ms, std::bind(&CanTxNode::can_tx_main_node_loop, this));
}

void CanTxNode::can_tx_main_node_loop() {
  can_frame frame = can_rx.receive();
  switch (frame.can_id) {
    // TODO: implement case when needed
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}
