#include "amk_startup.hpp"

using namespace PUTM_CAN;

 AmkFrontLeftSetpoints1 front_left_amk_setpoints{
        .AMK_Control = {0, 0, 0, 0, 0, 0},
        .AMK_TargetVelocity = 0,
        .AMK_TorqueLimitPositiv = 0,
        .AMK_TorqueLimitNegativ = 0,
  };

  AmkFrontRightSetpoints1 front_right_amk_setpoints{
        .AMK_Control = {0, 0, 0, 0, 0, 0},
        .AMK_TargetVelocity = 0,
        .AMK_TorqueLimitPositiv = 0,
        .AMK_TorqueLimitNegativ = 0,
  };

    enum states{
  STATE1,
  STATE2,
  STATE3,
  STATE4
};

CanTx can_tx("can0");

void SetTorque(uint16_t _torque)
{
    front_left_amk_setpoints.AMK_TorqueLimitNegativ = -300;
    front_left_amk_setpoints.AMK_TorqueLimitPositiv = 300;
    front_left_amk_setpoints.AMK_TargetVelocity = _torque;

    front_right_amk_setpoints.AMK_TorqueLimitNegativ = -300;
    front_right_amk_setpoints.AMK_TorqueLimitPositiv = 300;
    front_right_amk_setpoints.AMK_TargetVelocity = _torque;

    can_tx.transmit(front_left_amk_setpoints);
    can_tx.transmit(front_right_amk_setpoints);
}


void AMK_Startup()
{
  CanRx can_rx("can0", NO_TIMEOUT);

  states amk_state = states::STATE1;

  while (1) {

    can_frame frame = can_rx.receive();

    AmkFrontLeftActualValues1 front_left_values_1;
    AmkFrontRightActualValues1 front_right_values_1;

    if (frame.can_id == FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID)
    {
      front_left_values_1 = convert<AmkFrontLeftActualValues1>(frame);
    }
    else if (frame.can_id == FRONT_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID)
    {
      front_right_values_1 = convert<AmkFrontRightActualValues1>(frame);
    }
    switch (amk_state) 
      {
        case states::STATE1:
          if(front_left_values_1.AMK_Status.AMK_bSystemReady && front_right_values_1.AMK_Status.AMK_bSystemReady)
          {
              // Send AMK_bDcOn
              front_left_amk_setpoints.AMK_Control.AMK_bDcOn = 1;
              front_left_amk_setpoints.AMK_TorqueLimitNegativ = 0;
              front_left_amk_setpoints.AMK_TorqueLimitPositiv = 0;
              front_left_amk_setpoints.AMK_TargetVelocity = 0;

              front_right_amk_setpoints.AMK_Control.AMK_bDcOn = 1;
              front_right_amk_setpoints.AMK_TorqueLimitNegativ = 0;
              front_right_amk_setpoints.AMK_TorqueLimitPositiv = 0;
              front_right_amk_setpoints.AMK_TargetVelocity = 0;

              amk_state = states::STATE2;
              std::cout << "Both are ready, dc ON" << std::endl;
          }
        break;

        case states::STATE2:
        if(front_left_values_1.AMK_Status.AMK_bQuitDcOn && front_right_values_1.AMK_Status.AMK_bQuitDcOn)
        {
          front_left_amk_setpoints.AMK_Control.AMK_bEnable = 1;
          // front_left_amk_setpoints.AMK_Control.AMK_bInverterOn = 1;

          front_right_amk_setpoints.AMK_Control.AMK_bEnable = 1;
          // front_right_amk_setpoints.AMK_Control.AMK_bInverterOn = 1;

          std::cout << "Both are quit dc on, enabling inverter" << std::endl;

          amk_state = states::STATE3;
        }

        break;

        case states::STATE3:

          front_left_amk_setpoints.AMK_Control.AMK_bInverterOn = 1;
          front_right_amk_setpoints.AMK_Control.AMK_bInverterOn = 1;
          std::cout << "Both are enabled, enabling inverter" << std::endl;

        amk_state = states::STATE4;

        break;

        case states::STATE4:

        if(front_left_values_1.AMK_Status.AMK_bQuitInverterOn && front_right_values_1.AMK_Status.AMK_bQuitInverterOn)
        {
            return;
        }
        break;

        default:
          break;
    }
    can_tx.transmit(front_left_amk_setpoints);
    can_tx.transmit(front_right_amk_setpoints);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}
