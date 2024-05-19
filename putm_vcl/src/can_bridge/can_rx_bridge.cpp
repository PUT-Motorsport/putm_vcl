#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_headers/PM09-CANBUS-FRONTBOX.hpp"
#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/detail/frontbox__struct.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

using namespace std::chrono_literals;
using namespace PUTM_CAN;

class CanRxNode : public rclcpp::Node {
 public:
  CanRxNode();

 private:
  CanRx can_rx;
  rclcpp::TimerBase::SharedPtr can_rx_node_timer;
  rclcpp::Publisher<putm_vcl_interfaces::msg::Frontbox>::SharedPtr front_box_publisher;

  void can_rx_node_main_loop();
};

CanRxNode::CanRxNode() : Node("can_rx_node"), can_rx("can1", NO_TIMEOUT) {
  can_rx_node_timer = this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_node_main_loop, this));
  front_box_publisher = this->create_publisher<putm_vcl_interfaces::msg::Frontbox>("putm_vcl/frontbox", 1);
}

void CanRxNode::can_rx_node_main_loop() {
  can_frame frame = can_rx.receive();
  switch (frame.can_id) {
    case can_id<Frontbox_main>: {
      auto can_frontbox = PUTM_CAN::convert<PUTM_CAN::Frontbox_main>(frame);
      putm_vcl_interfaces::msg::Frontbox frontbox;
      frontbox.pedal_position = (((can_frontbox.pedal_position) / 500.0) * 100.0);
      front_box_publisher->publish(frontbox);
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}
