#pragma once

#include "can_headers/PM08-CANBUS-AMK-FRONT-LEFT.hpp"
#include "can_headers/PM08-CANBUS-AMK-FRONT-RIGHT.hpp"
#include "can_headers/PM08-CANBUS-APPS.hpp"
#include "can_headers/PM08-CANBUS-AQ_CARD.hpp"
#include "can_headers/PM08-CANBUS-BMS_HV.hpp"
#include "can_headers/PM08-CANBUS-BMS_LV.hpp"
#include "can_headers/PM08-CANBUS-DASH.hpp"
#include "can_headers/PM08-CANBUS-DV.hpp"
#include "can_headers/PM08-CANBUS-LAP_TIMER.hpp"
#include "can_headers/PM08-CANBUS-ODRIVE.hpp"
#include "can_headers/PM08-CANBUS-SF.hpp"
#include "can_headers/PM08-CANBUS-STEERING_WHEEL.hpp"
#include "can_headers/PM08-CANBUS-SW_SENSOR.hpp"
#include "can_headers/PM08-CANBUS-TC.hpp"
#include "can_headers/PM08-CANBUS-TELEMETRY.hpp"
#include "can_headers/PM08-CANBUS-WHEELTEMP.hpp"
#include "can_headers/PM08-CANBUS-YAWPROBE.hpp"

namespace PUTM_CAN {
static constexpr int INVALID_FILE_DESCRIPTOR = -1;
static constexpr uint16_t INVALID_CAN_ID = 0xFFFFU;

template <typename T>
constexpr uint16_t can_id = INVALID_CAN_ID;

template <>
constexpr uint16_t can_id<AmkFrontLeftActualValues1> = FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID;
template <>
constexpr uint16_t can_id<AmkFrontLeftActualValues2> = FRONT_LEFT_AMK_ACTUAL_VALUES_2_CAN_ID;
template <>
constexpr uint16_t can_id<AmkFrontLeftSetpoints1> = FRONT_LEFT_AMK_SETPOINTS_1_CAN_ID;

template <>
constexpr uint16_t can_id<AmkFrontRightActualValues1> = FRONT_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID;
template <>
constexpr uint16_t can_id<AmkFrontRightActualValues2> = FRONT_RIGHT_AMK_ACTUAL_VALUES_2_CAN_ID;
template <>
constexpr uint16_t can_id<AmkFrontRightSetpoints1> = FRONT_RIGHT_AMK_SETPOINTS_1_CAN_ID;

template <>
constexpr uint16_t can_id<Apps_main> = APPS_MAIN_CAN_ID;

template <>
constexpr uint16_t can_id<AQ_main> = AQ_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<AQ_acceleration> = AQ_ACCELERATION_CAN_ID;
template <>
constexpr uint16_t can_id<AQ_gyroscope> = AQ_GYROSCOPE_CAN_ID;
template <>
constexpr uint16_t can_id<AQ_ts_button> = AQ_TS_BUTTON_CAN_ID;

template <>
constexpr uint16_t can_id<BMS_HV_main> = BMS_HV_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<BMS_LV_main> = BMS_LV_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<BMS_LV_temperature> = BMS_LV_TEMPERATURE_CAN_ID;

template <>
constexpr uint16_t can_id<Dash_Main> = DASH_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<Dash_TCS> = DASH_TCS_CAN_ID;
template <>
constexpr uint16_t can_id<Dash_Smart_Fuses_FAN_speed> = DASH_SMART_FUSES_FAN_SPEED_CAN_ID;
template <>
constexpr uint16_t can_id<Dash_steering_wheel_request> = DASH_STEERING_WHEEL_REQUEST_CAN_ID;
template <>
constexpr uint16_t can_id<Dash_lap_finished> = DASH_LAP_FINISHED_CAN_ID;
template <>
constexpr uint16_t can_id<Dash_steering_wheel_angle> = DASH_STEERING_WHEEL_ANGLE_CAN_ID;

template <>
constexpr uint16_t can_id<DV_Ass> = DV_ASS_CAN_ID;

template <>
constexpr uint16_t can_id<Lap_timer_Main> = LAP_TIMER_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<Lap_timer_Sector> = LAP_TIMER_SECTOR_CAN_ID;
template <>
constexpr uint16_t can_id<Lap_timer_Acc_time> = LAP_TIMER_ACC_TIME_CAN_ID;
template <>
constexpr uint16_t can_id<Lap_timer_Skidpad_time> = LAP_TIMER_SKIDPAD_TIME_CAN_ID;
template <>
constexpr uint16_t can_id<Lap_timer_Lap_time> = LAP_TIMER_LAP_TIME_CAN_ID;

template <>
constexpr uint16_t can_id<SF_main> = SF_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<SF_PassiveElements> = SF_PASSIVEELEMENTS_CAN_ID;
template <>
constexpr uint16_t can_id<SF_LegendaryDVAndSupply> = SF_LEGENDARYDVANDSUPPLY_CAN_ID;
template <>
constexpr uint16_t can_id<SF_Supply> = SF_SUPPLY_CAN_ID;
template <>
constexpr uint16_t can_id<SF_safety> = SF_SAFETY_CAN_ID;

template <>
constexpr uint16_t can_id<Steering_Wheel_main> = STEERING_WHEEL_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<Steering_Wheel_event> = STEERING_WHEEL_EVENT_CAN_ID;

template <>
constexpr uint16_t can_id<SWPS_main> = SWPS_MAIN_CAN_ID;

template <>
constexpr uint16_t can_id<TC_main> = TC_MAIN_CAN_ID;
template <>
constexpr uint16_t can_id<TC_rear_suspension> = TC_REAR_SUSPENSION_CAN_ID;
template <>
constexpr uint16_t can_id<TC_wheel_velocities> = TC_WHEEL_VELOCITIES_CAN_ID;
template <>
constexpr uint16_t can_id<TC_temperatures> = TC_TEMPERATURES_CAN_ID;
template <>
constexpr uint16_t can_id<TC_imu_gyro> = TC_IMU_GYRO_CAN_ID;
template <>
constexpr uint16_t can_id<TC_imu_acc> = TC_IMU_ACC_CAN_ID;

template <>
constexpr uint16_t can_id<Telemetry_Main> = TELEMETRY_MAIN_CAN_ID;

template <>
constexpr uint16_t can_id<WheelTemp_main> = WHEELTEMP_MAIN_CAN_ID;

template <>
constexpr uint16_t can_id<YawProbe_air_flow> = YAWPROBE_AIR_FLOW_CAN_ID;

template <>
constexpr uint16_t can_id<Odrive_Heartbeat> = ODRIVE_HEARTBEAT_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Estop> = ODRIVE_ESTOP_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Motor_Error> = ODRIVE_GET_MOTOR_ERROR_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Encoder_Error> = ODRIVE_GET_ENCODER_ERROR_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Axis_Node_ID> = ODRIVE_SET_AXIS_NODE_ID_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Axis_State> = ODRIVE_SET_AXIS_STATE_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Encoder_Estimation> = ODRIVE_GET_ENCODER_ESTIMATION_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Encoder_Count> = ODRIVE_GET_ENCODER_COUNT_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Controller_Mode> = ODRIVE_SET_CONTROLLER_MODE_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Input_Position> = ODRIVE_SET_INPUT_POSITION_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Input_Vel> = ODRIVE_SET_INPUT_VEL_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Input_Torque> = ODRIVE_SET_INPUT_TORQUE_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Limits> = ODRIVE_SET_LIMITS_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Start_Anticogging> = ODRIVE_START_ANTICOGGING_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Traj_Vel_Limit> = ODRIVE_SET_TRAJ_VEL_LIMIT_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Traj_Accel_Limits> = ODRIVE_SET_TRAJ_ACCEL_LIMITS_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Traj_Inertia> = ODRIVE_SET_TRAJ_INERTIA_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Iq> = ODRIVE_GET_IQ_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Temperature> = ODRIVE_GET_TEMPERATURE_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Reboot> = ODRIVE_REBOOT_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Vbus_Voltage_Current> = ODRIVE_GET_VBUS_VOLTAGE_CURRENT_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Clear_Errors> = ODRIVE_CLEAR_ERRORS_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Absolute_Position> = ODRIVE_SET_ABSOLUTE_POSITION_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Pos_Gain> = ODRIVE_SET_POS_GAIN_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Set_Vel_gains> = ODRIVE_SET_VEL_GAINS_CAN_ID;
template <>
constexpr uint16_t can_id<Odrive_Get_Controller_Error> = ODRIVE_GET_CONTROLLER_ERROR_CAN_ID;

}  // namespace PUTM_CAN
