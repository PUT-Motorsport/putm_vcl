#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"
#include "putm_vcl_interfaces/msg/amk_data.hpp"
#include "putm_vcl_interfaces/msg/amk_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace PUTM_CAN;

class AmkRxNode : public rclcpp::Node {
 public:
  AmkRxNode();

 private:
  enum Inverters { FRONT_LEFT = 0, FRONT_RIGHT = 1, REAR_LEFT = 2, REAR_RIGHT = 3 };

  CanRx can_rx;
  putm_vcl_interfaces::msg::AmkStatus amk_status;
  putm_vcl_interfaces::msg::AmkData amk_data;
  rclcpp::TimerBase::SharedPtr amk_rx_node_timer;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkStatus>::SharedPtr amk_status_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkData>::SharedPtr amk_data_publisher;

  void amk_rx_node_main_loop();
};

AmkRxNode::AmkRxNode() : Node("amk_rx_node"), can_rx("can0", NO_TIMEOUT) {
  amk_rx_node_timer = this->create_wall_timer(1ms, std::bind(&AmkRxNode::amk_rx_node_main_loop, this));
  amk_status_publisher = this->create_publisher<putm_vcl_interfaces::msg::AmkStatus>("putm_vcl/amk_status", 1);
  amk_data_publisher = this->create_publisher<putm_vcl_interfaces::msg::AmkData>("putm_vcl/amk_data", 1);
}

// TODO: Change case define to can_id
void AmkRxNode::amk_rx_node_main_loop() {
  can_frame frame = can_rx.receive();
  switch (frame.can_id) {
    case FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto frontLeftAmkActualValues1 = convert<AmkFrontLeftActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bDerating;

      amk_data.amk_actual_velocity[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_ActualVelocity;
      amk_data.amk_torque_current[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_TorqueCurrent;
    } break;

    case FRONT_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto frontRightAmkActualValues1 = convert<AmkFrontRightActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bDerating;
    } break;

    case REAR_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto rearLeftAmkActualValues1 = convert<AmkRearLeftActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bDerating;
    } break;

    case REAR_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto rearRightAmkActualValues1 = convert<AmkRearRightActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bDerating;
    } break;
  }
  amk_status_publisher->publish(amk_status);
  amk_data_publisher->publish(amk_data);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmkRxNode>());
  rclcpp::shutdown();
}
