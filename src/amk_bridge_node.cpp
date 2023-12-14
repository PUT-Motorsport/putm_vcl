#include <cstdio>

#include "can_library/can_rx.hpp"
#include "can_library/can_tx.hpp"

#include "rclcpp/rclcpp.hpp"
// #include "putm_ev_amk_2023/msg/front.hpp"

using namespace PUTM_CAN;

// rclcpp::Publisher<putm_ev_amk_2023::msg::Front>::SharedPtr FrontLeftInverterpublisher_;
// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr FrontRightInverterpublisher_;
// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RearLeftInverterpublisher_;
// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RearRightInverterpublisher_;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  CanTx can_tx("can0");
  CanRx can_rx("can0", NO_TIMEOUT);

  while(true)
  {
    can_frame frame = can_rx.receive();
    switch(frame.can_id)
    {
      case FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID:

      break;

      case FRONT_LEFT_AMK_ACTUAL_VALUES_2_CAN_ID:

      break;

      case FRONT_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID:

      break;

      case FRONT_RIGHT_AMK_ACTUAL_VALUES_2_CAN_ID:

      break;
    }
  }
}