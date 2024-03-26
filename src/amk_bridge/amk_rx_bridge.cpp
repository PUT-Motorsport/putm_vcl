#include <cstdio>

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_tx.hpp"

#include "rclcpp/rclcpp.hpp"
#include "putm_pm09_vcl/msg/amk_status.hpp"
#include "putm_pm09_vcl/msg/amk_data.hpp"

using namespace PUTM_CAN;

enum Inverters : size_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("amk_bridge_node");
  rclcpp::Publisher<putm_pm09_vcl::msg::AmkStatus>::SharedPtr AmkStatusPublisher = node->create_publisher<putm_pm09_vcl::msg::AmkStatus>("amk_status", 10);
  rclcpp::Publisher<putm_pm09_vcl::msg::AmkData>  ::SharedPtr AmkDataPublisher   = node->create_publisher<putm_pm09_vcl::msg::AmkData>  ("amk_data",   10);

  (void) argc;
  (void) argv;

  CanRx can_rx("can0", NO_TIMEOUT);

  putm_pm09_vcl::msg::AmkStatus amk_status;
  putm_pm09_vcl::msg::AmkData amk_data;

  while(true)
  {
    can_frame frame = can_rx.receive();
    switch(frame.can_id)
    {
      case FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID:
      {
        auto frontLeftAmkActualValues1 = convert<AmkFrontLeftActualValues1>(frame);
        amk_status.amk_status_bsystem_ready     [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bSystemReady;
        amk_status.amk_status_berror            [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bError;
        amk_status.amk_status_bwarn             [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bWarn;
        amk_status.amk_status_bquit_dc_on       [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
        amk_status.amk_status_bdc_on            [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bDcOn;
        amk_status.amk_status_bquit_inverter_on [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
        amk_status.amk_status_binverter_on      [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bInverterOn;
        amk_status.amk_status_bderating         [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bDerating;

        amk_data.amk_actual_velocity            [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_ActualVelocity;
        amk_data.amk_torque_current             [Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_TorqueCurrent;
      }
      break;

      case FRONT_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID:
      {
        auto frontRightAmkActualValues1 = convert<AmkFrontRightActualValues1>(frame);
        amk_status.amk_status_bsystem_ready     [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bSystemReady;
        amk_status.amk_status_berror            [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bError;
        amk_status.amk_status_bwarn             [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bWarn;
        amk_status.amk_status_bquit_dc_on       [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
        amk_status.amk_status_bdc_on            [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bDcOn;
        amk_status.amk_status_bquit_inverter_on [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
        amk_status.amk_status_binverter_on      [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bInverterOn;
        amk_status.amk_status_bderating         [Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bDerating;
      }
      break;

      case REAR_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID:
      {
        auto rearLeftAmkActualValues1 = convert<AmkRearLeftActualValues1>(frame);
        amk_status.amk_status_bsystem_ready     [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bSystemReady;
        amk_status.amk_status_berror            [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bError;
        amk_status.amk_status_bwarn             [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bWarn;
        amk_status.amk_status_bquit_dc_on       [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
        amk_status.amk_status_bdc_on            [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bDcOn;
        amk_status.amk_status_bquit_inverter_on [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
        amk_status.amk_status_binverter_on      [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bInverterOn;
        amk_status.amk_status_bderating         [Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bDerating;
      }
      break;

      case REAR_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID:
      {
        auto rearRightAmkActualValues1 = convert<AmkRearRightActualValues1>(frame);
        amk_status.amk_status_bsystem_ready     [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bSystemReady;
        amk_status.amk_status_berror            [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bError;
        amk_status.amk_status_bwarn             [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bWarn;
        amk_status.amk_status_bquit_dc_on       [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
        amk_status.amk_status_bdc_on            [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bDcOn;
        amk_status.amk_status_bquit_inverter_on [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
        amk_status.amk_status_binverter_on      [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bInverterOn;
        amk_status.amk_status_bderating         [Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bDerating;
      }
      break;
    }
    AmkStatusPublisher->publish(amk_status);
    AmkDataPublisher  ->publish(amk_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));


  }
}
