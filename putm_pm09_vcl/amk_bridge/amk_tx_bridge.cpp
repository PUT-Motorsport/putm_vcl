#include <cstdio>

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"

#include "rclcpp/rclcpp.hpp"

#include "putm_pm09_vcl/msg/amk_control.hpp"

using namespace PUTM_CAN;

using std::placeholders::_1;
using namespace std::chrono_literals;

CanTx can_tx("can0");

// void topic_callback(const putm_pm09_vcl::msg::AmkControl msg)
// {
//     AmkFrontLeftSetpoints1 front_left_amk_setpoints{
//         .AMK_Control = {0, msg.amk_control_binverter_on[0], msg.amk_control_bdc_on[0], msg.amk_control_benable[0], msg.amk_control_amkb_error_reset[0], 0},
//         .AMK_TargetVelocity = msg.amk_target_tourqe[0],
//         .AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[0],
//         .AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[0],
//     };
//     can_tx.transmit(front_left_amk_setpoints);

//     AmkFrontRightSetpoints1 front_right_amk_setpoints{
//         .AMK_Control = {0, msg.amk_control_binverter_on[1], msg.amk_control_bdc_on[1], msg.amk_control_benable[1], msg.amk_control_amkb_error_reset[1], 0},
//         .AMK_TargetVelocity = msg.amk_target_tourqe[1],
//         .AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[1],
//         .AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[1],
//     };
//     can_tx.transmit(front_right_amk_setpoints);

//     AmkRearLeftSetpoints1 rear_left_amk_setpoints{
//         .AMK_Control = {0, msg.amk_control_binverter_on[2], msg.amk_control_bdc_on[2], msg.amk_control_benable[2], msg.amk_control_amkb_error_reset[2], 0},
//         .AMK_TargetVelocity = msg.amk_target_tourqe[2],
//         .AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[2],
//         .AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[2],
//     };
//     can_tx.transmit(rear_left_amk_setpoints);

//     AmkRearRightSetpoints1 rear_right_amk_setpoints{
//         .AMK_Control = {0, msg.amk_control_binverter_on[3], msg.amk_control_bdc_on[3], msg.amk_control_benable[3], msg.amk_control_amkb_error_reset[3], 0},
//         .AMK_TargetVelocity = msg.amk_target_tourqe[3],
//         .AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[3],
//         .AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[3],
//     };
//     can_tx.transmit(rear_right_amk_setpoints);
// }
AmkFrontLeftSetpoints1 front_left_amk_setpoints {};
AmkFrontRightSetpoints1 front_right_amk_setpoints {};
AmkRearLeftSetpoints1 rear_left_amk_setpoints{};
AmkRearRightSetpoints1 rear_right_amk_setpoints {};


class AmkBridgeTxNode : public rclcpp::Node
{
  public:
    AmkBridgeTxNode()
    : Node("amk_tx_bridge")
    {
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
    // can_tx.transmit(front_left_amk_setpoints);

        front_right_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[1], msg.amk_control_bdc_on[1], msg.amk_control_benable[1], msg.amk_control_amkb_error_reset[1], 0};
        front_right_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[1];
        front_right_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[1];
        front_right_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[1];
    // can_tx.transmit(front_right_amk_setpoints);

        rear_left_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[2], msg.amk_control_bdc_on[2], msg.amk_control_benable[2], msg.amk_control_amkb_error_reset[2], 0};
        rear_left_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[2];
        rear_left_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[2];
        rear_left_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[2];
    // can_tx.transmit(rear_left_amk_setpoints);

        rear_right_amk_setpoints.AMK_Control = {0, msg.amk_control_binverter_on[3], msg.amk_control_bdc_on[3], msg.amk_control_benable[3], msg.amk_control_amkb_error_reset[3], 0};
        rear_right_amk_setpoints.AMK_TargetVelocity = msg.amk_target_tourqe[3];
        rear_right_amk_setpoints.AMK_TorqueLimitPositiv = msg.amk_tourqe_positive_limit[3];
        rear_right_amk_setpoints.AMK_TorqueLimitNegativ = msg.amk_tourqe_negative_limit[3];
    // can_tx.transmit(rear_right_amk_setpoints);
    }
    void amk_can_tx()
    {
        can_tx.transmit(front_left_amk_setpoints);
        can_tx.transmit(front_right_amk_setpoints);
        can_tx.transmit(rear_left_amk_setpoints);
        can_tx.transmit(rear_right_amk_setpoints);
    }
    rclcpp::Subscription<putm_pm09_vcl::msg::AmkControl>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<rclcpp::Node>("amk_bridge_tx_node");
    // rclcpp::Subscription<putm_pm09_vcl::msg::AmkControl>::SharedPtr AmkControlSubscriber = node->create_subscription<putm_ev_amk_2023::msg::AmkControl>("amk_control", 10, std::bind(&topic_callback, std::placeholders::_1));
    rclcpp::spin(std::make_shared<AmkBridgeTxNode>());
    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
} 
