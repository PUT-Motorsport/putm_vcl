#include <cstdio>
#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_pm09_vcl/msg/amk_control.hpp"

using namespace PUTM_CAN;

using std::placeholders::_1;
using namespace std::chrono_literals;

AmkFrontLeftSetpoints1 front_left_amk_setpoints = {amkFrontLeftControl_t{0,0,0,0,0,0}, 0, 0, 0}; 
AmkFrontRightSetpoints1 front_right_amk_setpoints = {amkFrontRightControl_t{0,0,0,0,0,0}, 0, 0, 0}; 
AmkRearLeftSetpoints1 rear_left_amk_setpoints = {amkRearLeftControl_t{0,0,0,0,0,0}, 0, 0, 0};   
AmkRearRightSetpoints1 rear_right_amk_setpoints = {amkRearRightControl_t{0,0,0,0,0,0}, 0, 0, 0}; 
 
class AmkBridgeTxNode : public rclcpp::Node
{
  public:
    AmkBridgeTxNode() : Node("amk_tx_bridge"), can_tx("can0")
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "My log message %d", 4);
      subscription_ = this->create_subscription<putm_pm09_vcl::msg::AmkControl>("amk_control", 10, std::bind(&AmkBridgeTxNode::amk_control_callback, this, _1));
      timer_ = this->create_wall_timer(10ms, std::bind(&AmkBridgeTxNode::amk_can_tx, this));
    }
  private:
    void amk_control_callback(const putm_pm09_vcl::msg::AmkControl msg) const
    {
        front_left_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[0], msg.amk_control_bdc_on[0], msg.amk_control_benable[0], msg.amk_control_amkb_error_reset[0], 0};
        front_left_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[0];
        front_left_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[0];
        front_left_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[0];
    
        front_right_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[1], msg.amk_control_bdc_on[1], msg.amk_control_benable[1], msg.amk_control_amkb_error_reset[1], 0};
        front_right_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[1];
        front_right_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[1];
        front_right_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[1];
   
        rear_left_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[2], msg.amk_control_bdc_on[2], msg.amk_control_benable[2], msg.amk_control_amkb_error_reset[2], 0};
        rear_left_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[2];
        rear_left_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[2];
        rear_left_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[2];

        rear_right_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[3], msg.amk_control_bdc_on[3], msg.amk_control_benable[3], msg.amk_control_amkb_error_reset[3], 0};
        rear_right_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[3];
        rear_right_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[3];
        rear_right_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[3];

    }
    void amk_can_tx()
    {
        can_tx.transmit(front_left_amk_setpoints);
        can_tx.transmit(front_right_amk_setpoints);
        can_tx.transmit(rear_left_amk_setpoints);
        can_tx.transmit(rear_right_amk_setpoints);
    }
    CanTx can_tx;
    rclcpp::Subscription<putm_pm09_vcl::msg::AmkControl>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AmkBridgeTxNode>());
    rclcpp::shutdown();

    return 0;
} 
