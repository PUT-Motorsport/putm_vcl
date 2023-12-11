//Generated on 2023-03-10 21:36:48.871086
#pragma once

#include <cstdint>

namespace PUTM_CAN{

	enum struct Odrive_states: uint8_t {
	};

	struct __attribute__ ((packed)) Odrive_Heartbeat{
		uint32_t Axis_Error;//Axis error
		uint8_t Axis_State;//Axis State
		uint8_t Flags;
	};

	struct __attribute__ ((packed)) Odrive_Estop{
	};

	struct __attribute__ ((packed)) Odrive_Get_Motor_Error{
		uint8_t Active_Errors;
		uint8_t Disarm_Reason;
	};

	struct __attribute__ ((packed)) Odrive_Get_Encoder_Error{
		uint8_t Active_Errors;
		uint8_t Disarm_Reason;
	};

	struct __attribute__ ((packed)) Odrive_Set_Axis_Node_ID{
		uint8_t Axis_Node_ID;
	};

	struct __attribute__ ((packed)) Odrive_Set_Axis_State{
		uint32_t Axis_Requested_State;
	};

	struct __attribute__ ((packed)) Odrive_Get_Encoder_Estimation{
		float Pos_Estimate;
		float Vel_Estimate;
	};

	struct __attribute__ ((packed)) Odrive_Get_Encoder_Count{
		float Shadow_Count;
		float Count_in_CPR;
	};

	struct __attribute__ ((packed)) Odrive_Set_Controller_Mode{
		int32_t Control_Mode;
		int32_t Input_Mode;
	};

	struct __attribute__ ((packed)) Odrive_Set_Input_Position{
		float Input_Pos;
		int16_t Vel_FF;
		int16_t Torque_FF;
	};

	struct __attribute__ ((packed)) Odrive_Set_Input_Vel{
		float Input_Vel;
		float Input_Torque_FF;
	};

	struct __attribute__ ((packed)) Odrive_Set_Input_Torque{
		float Input_Torque;
	};

	struct __attribute__ ((packed)) Odrive_Set_Limits{
		float Velocity_Limit;
		float Current_Limit;
	};

	struct __attribute__ ((packed)) Odrive_Start_Anticogging{
	};

	struct __attribute__ ((packed)) Odrive_Set_Traj_Vel_Limit{
		float Traj_Vel_Limit;
	};

	struct __attribute__ ((packed)) Odrive_Set_Traj_Accel_Limits{
		float Traj_Accel_Limit;
		float Traj_Decel_Limit;
	};

	struct __attribute__ ((packed)) Odrive_Set_Traj_Inertia{
		float Traj_Inertia;
	};

	struct __attribute__ ((packed)) Odrive_Get_Iq{
		float Iq_Setpoint;
		float Iq_Measured;
	};

	struct __attribute__ ((packed)) Odrive_Get_Temperature{
		float FET_Temperature;
		float Motor_Temperature;
	};

	struct __attribute__ ((packed)) Odrive_Reboot{
	};

	struct __attribute__ ((packed)) Odrive_Get_Vbus_Voltage_Current{
		float Vbus_Voltage;
		float Vbus_Current;
	};

	struct __attribute__ ((packed)) Odrive_Clear_Errors{
	};

	struct __attribute__ ((packed)) Odrive_Set_Absolute_Position{
		float Position;
	};

	struct __attribute__ ((packed)) Odrive_Set_Pos_Gain{
		float Pos_Gain;
	};

	struct __attribute__ ((packed)) Odrive_Set_Vel_gains{
		float Vel_Gain;
		float Vel_Integrator_Gain;
	};
	struct __attribute__ ((packed)) Odrive_Get_Controller_Error{
		uint8_t Active_Errors;
		uint8_t Disarm_Reason;
	};

	const uint16_t ODRIVE_HEARTBEAT_CAN_ID = 0xA1;
	const uint8_t ODRIVE_HEARTBEAT_CAN_DLC = sizeof(Odrive_Heartbeat);
	const uint8_t ODRIVE_HEARTBEAT_FREQUENCY = 1;
	const uint16_t ODRIVE_ESTOP_CAN_ID = 0xA2;
	const uint8_t ODRIVE_ESTOP_CAN_DLC = sizeof(Odrive_Estop);
	const uint8_t ODRIVE_ESTOP_FREQUENCY = 0;
	
	const uint16_t ODRIVE_GET_MOTOR_ERROR_CAN_ID = 0xA3;
	const uint8_t ODRIVE_GET_MOTORERROR_CAN_DLC = sizeof(Odrive_Get_Motor_Error);
	const uint8_t ODRIVE_GET_MOTOR_ERROR_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_ENCODER_ERROR_CAN_ID = 0xA4;
	const uint8_t ODRIVE_GET_ENCODER_ERROR_CAN_DLC = sizeof(Odrive_Get_Encoder_Error);
	const uint8_t ODRIVE_GET_ENCODER_ERROR_FREQUENCY = 0;

	const uint16_t ODRIVE_SET_AXIS_NODE_ID_CAN_ID = 0xA6;
	const uint8_t ODRIVE_SET_AXIS_NODE_ID_CAN_DLC = sizeof(Odrive_Set_Axis_Node_ID);
	const uint8_t ODRIVE_SET_AXIS_NODE_ID_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_AXIS_STATE_CAN_ID = 0xA7;
	const uint8_t ODRIVE_SET_AXIS_STATE_CAN_DLC = sizeof(Odrive_Set_Axis_State);
	const uint8_t ODRIVE_SET_AXIS_STATE_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_ENCODER_ESTIMATION_CAN_ID = 0xA9;
	const uint8_t ODRIVE_GET_ENCODER_ESTIMATION_CAN_DLC = sizeof(Odrive_Get_Encoder_Estimation);
	const uint8_t ODRIVE_GET_ENCODER_ESTIMATION_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_ENCODER_COUNT_CAN_ID = 0xAA;
	const uint8_t ODRIVE_GET_ENCODER_COUNT_CAN_DLC = sizeof(Odrive_Get_Encoder_Count);
	const uint8_t ODRIVE_GET_ENCODER_COUNT_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_CONTROLLER_MODE_CAN_ID = 0xAB;
	const uint8_t ODRIVE_SET_CONTROLLER_MODE_CAN_DLC = sizeof(Odrive_Set_Controller_Mode);
	const uint8_t ODRIVE_SET_CONTROLLER_MODE_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_INPUT_POSITION_CAN_ID = 0xAC;
	const uint8_t ODRIVE_SET_INPUT_POSITION_CAN_DLC = sizeof(Odrive_Set_Input_Position);
	const uint8_t ODRIVE_SET_INPUT_POSITION_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_INPUT_VEL_CAN_ID = 0xAD;
	const uint8_t ODRIVE_SET_INPUT_VEL_CAN_DLC = sizeof(Odrive_Set_Input_Vel);
	const uint8_t ODRIVE_SET_INPUT_VEL_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_INPUT_TORQUE_CAN_ID = 0xAE;
	const uint8_t ODRIVE_SET_INPUT_TORQUE_CAN_DLC = sizeof(Odrive_Set_Input_Torque);
	const uint8_t ODRIVE_SET_INPUT_TORQUE_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_LIMITS_CAN_ID = 0xAF;
	const uint8_t ODRIVE_SET_LIMITS_CAN_DLC = sizeof(Odrive_Set_Limits);
	const uint8_t ODRIVE_SET_LIMITS_FREQUENCY = 0;
	const uint16_t ODRIVE_START_ANTICOGGING_CAN_ID = 0xB0;
	const uint8_t ODRIVE_START_ANTICOGGING_CAN_DLC = sizeof(Odrive_Start_Anticogging);
	const uint8_t ODRIVE_START_ANTICOGGING_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_TRAJ_VEL_LIMIT_CAN_ID = 0xB1;
	const uint8_t ODRIVE_SET_TRAJ_VEL_LIMIT_CAN_DLC = sizeof(Odrive_Set_Traj_Vel_Limit);
	const uint8_t ODRIVE_SET_TRAJ_VEL_LIMIT_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_TRAJ_ACCEL_LIMITS_CAN_ID = 0xB2;
	const uint8_t ODRIVE_SET_TRAJ_ACCEL_LIMITS_CAN_DLC = sizeof(Odrive_Set_Traj_Accel_Limits);
	const uint8_t ODRIVE_SET_TRAJ_ACCEL_LIMITS_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_TRAJ_INERTIA_CAN_ID = 0xB3;
	const uint8_t ODRIVE_SET_TRAJ_INERTIA_CAN_DLC = sizeof(Odrive_Set_Traj_Inertia);
	const uint8_t ODRIVE_SET_TRAJ_INERTIA_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_IQ_CAN_ID = 0xB4;
	const uint8_t ODRIVE_GET_IQ_CAN_DLC = sizeof(Odrive_Get_Iq);
	const uint8_t ODRIVE_GET_IQ_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_TEMPERATURE_CAN_ID = 0xB5;
	const uint8_t ODRIVE_GET_TEMPERATURE_CAN_DLC = sizeof(Odrive_Get_Temperature);
	const uint8_t ODRIVE_GET_TEMPERATURE_FREQUENCY = 0;
	const uint16_t ODRIVE_REBOOT_CAN_ID = 0xB6;
	const uint8_t ODRIVE_REBOOT_CAN_DLC = sizeof(Odrive_Reboot);
	const uint8_t ODRIVE_REBOOT_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_VBUS_VOLTAGE_CURRENT_CAN_ID = 0xB7;
	const uint8_t ODRIVE_GET_VBUS_VOLTAGE_CURRENT_CAN_DLC = sizeof(Odrive_Get_Vbus_Voltage_Current);
	const uint8_t ODRIVE_GET_VBUS_VOLTAGE_CURRENT_FREQUENCY = 0;
	const uint16_t ODRIVE_CLEAR_ERRORS_CAN_ID = 0xB8;
	const uint8_t ODRIVE_CLEAR_ERRORS_CAN_DLC = sizeof(Odrive_Clear_Errors);
	const uint8_t ODRIVE_CLEAR_ERRORS_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_ABSOLUTE_POSITION_CAN_ID = 0xB9;
	const uint8_t ODRIVE_SET_ABSOLUTE_POSITION_CAN_DLC = sizeof(Odrive_Set_Absolute_Position);
	const uint8_t ODRIVE_SET_ABSOLUTE_POSITION_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_POS_GAIN_CAN_ID = 0xBA;
	const uint8_t ODRIVE_SET_POS_GAIN_CAN_DLC = sizeof(Odrive_Set_Pos_Gain);
	const uint8_t ODRIVE_SET_POS_GAIN_FREQUENCY = 0;
	const uint16_t ODRIVE_SET_VEL_GAINS_CAN_ID = 0xBB;
	const uint8_t ODRIVE_SET_VEL_GAINS_CAN_DLC = sizeof(Odrive_Set_Vel_gains);
	const uint8_t ODRIVE_SET_VEL_GAINS_FREQUENCY = 0;
	const uint16_t ODRIVE_GET_CONTROLLER_ERROR_CAN_ID = 0xBC;
	const uint8_t ODRIVE_GET_CONTROLLER_ERROR_CAN_DLC = sizeof(Odrive_Get_Controller_Error);
	const uint8_t ODRIVE_GET_CONTROLLER_ERROR_FREQUENCY = 0;

}

