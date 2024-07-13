#include "can_nodes/can_rx_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

CanRxNode::CanRxNode()
    : Node("can_rx_node"),
      can_rx_amk(can_interface_amk, NO_TIMEOUT),
      can_rx_common(can_interface_common, NO_TIMEOUT),
      can_rx_amk_timer(this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_amk_callback, this))),
      can_rx_common_timer(this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_common_callback, this))),

      frontbox_driver_input_publisher(this->create_publisher<msg::FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1)),
      frontbox_data_publisher(this->create_publisher<msg::FrontboxData>("putm_vcl/frontbox_data", 1)),

      amk_front_left_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("putm_vcl/amk/front/left/actual_values1", 1)),
      amk_front_left_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("putm_vcl/amk/front/left/actual_values2", 1)),

      amk_front_right_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("putm_vcl/amk/front/right/actual_values1", 1)),
      amk_front_right_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("putm_vcl/amk/front/right/actual_values2", 1)),

      amk_rear_left_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("putm_vcl/amk/rear/left/actual_values1", 1)),
      amk_rear_left_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("putm_vcl/amk/rear/left/actual_values2", 1)),

      amk_rear_right_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("putm_vcl/amk/rear/right/actual_values1", 1)),
      amk_rear_right_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("putm_vcl/amk/rear/right/actual_values2", 1)),

      dashboard_publisher(this->create_publisher<msg::Dashboard>("putm_vcl/dashboard", 1)) {}
void CanRxNode::can_rx_common_callback() {
  can_frame frame = can_rx_common.receive();
  switch (frame.can_id) {
    case can_id<FrontboxDriverInput>: {
      auto can_frontbox_driver_input = convert<FrontboxDriverInput>(frame);
      msg::FrontboxDriverInput frontbox_driver_input;
      frontbox_driver_input.pedal_position = can_frontbox_driver_input.pedal_position;
      frontbox_driver_input.brake_pressure_front = can_frontbox_driver_input.brake_pressure_front;
      frontbox_driver_input.brake_pressure_rear = can_frontbox_driver_input.brake_pressure_rear;
      frontbox_driver_input.steering_wheel_position = can_frontbox_driver_input.steering_wheel_position;
      frontbox_driver_input_publisher->publish(frontbox_driver_input);
      break;
    }

    case can_id<FrontboxData>: {
      auto can_frontbox_data = convert<FrontboxData>(frame);
      msg::FrontboxData frontbox_data;
      frontbox_data.sense_left_kill = can_frontbox_data.sense_left_kill;
      frontbox_data.sense_right_kill = can_frontbox_data.sense_right_kill;
      frontbox_data.sense_driver_kill = can_frontbox_data.sense_driver_kill;
      frontbox_data.sense_inertia = can_frontbox_data.sense_inertia;
      frontbox_data.sense_bspd = can_frontbox_data.sense_bspd;
      frontbox_data.sense_overtravel = can_frontbox_data.sense_overtravel;
      frontbox_data.sense_right_wheel = can_frontbox_data.sense_right_wheel;
      frontbox_data.sc_state = can_frontbox_data.sc_state;
      frontbox_data.front_left_suspension = can_frontbox_data.front_left_suspension;
      frontbox_data.front_right_suspension = can_frontbox_data.front_right_suspension;
      frontbox_data.front_left_hub_temperature = can_frontbox_data.front_left_hub_temperature;
      frontbox_data.front_right_hub_temperature = can_frontbox_data.front_right_hub_temperature;
      frontbox_data_publisher->publish(frontbox_data);
      break;
    }

    case can_id<Dashboard>: {
      auto can_dashboard = convert<Dashboard>(frame);
      msg::Dashboard dashboard;
      dashboard.rtd_button = can_dashboard.rtd_button;
      dashboard.ts_activate_button = can_dashboard.ts_activate_button;
      dashboard.rfu_button = can_dashboard.rfu_button;
      dashboard_publisher->publish(dashboard);
      break;
    }
  }
}

void CanRxNode::can_rx_amk_callback() {
  can_frame frame = can_rx_amk.receive();
  msg::AmkActualValues1 amk_actual_values1;
  msg::AmkActualValues2 amk_actual_values2;

  switch (frame.can_id) {
    case can_id<AmkFrontLeftActualValues1>: {
      auto can_amk = convert<AmkFrontLeftActualValues1>(frame);
      amk_actual_values1.amk_status.system_ready = can_amk.amk_status.system_ready;
      amk_actual_values1.amk_status.error = can_amk.amk_status.error;
      amk_actual_values1.amk_status.warn = can_amk.amk_status.warn;
      amk_actual_values1.amk_status.quit_dc_on = can_amk.amk_status.quit_dc_on;
      amk_actual_values1.amk_status.dc_on = can_amk.amk_status.dc_on;
      amk_actual_values1.amk_status.quit_inverter_on = can_amk.amk_status.quit_inverter_on;
      amk_actual_values1.amk_status.inverter_on = can_amk.amk_status.inverter_on;
      amk_actual_values1.amk_status.derating = can_amk.amk_status.derating;
      amk_actual_values1.actual_velocity = can_amk.actual_velocity;
      amk_actual_values1.torque_current = can_amk.torque_current;
      amk_actual_values1.magnetizing_current = can_amk.magnetizing_current;
      amk_front_left_actual_values1_publisher->publish(amk_actual_values1);
      break;
    }

    case can_id<AmkFrontLeftActualValues2>: {
      auto can_amk = convert<AmkFrontLeftActualValues2>(frame);
      amk_actual_values2.temp_motor = can_amk.temp_motor;
      amk_actual_values2.temp_inverter = can_amk.temp_inverter;
      amk_actual_values2.error_info = can_amk.error_info;
      amk_actual_values2.temp_igbt = can_amk.temp_igbt;
      amk_front_left_actual_values2_publisher->publish(amk_actual_values2);
      break;
    }

    case can_id<AmkFrontRightActualValues1>: {
      auto can_amk = convert<AmkFrontRightActualValues1>(frame);
      amk_actual_values1.amk_status.system_ready = can_amk.amk_status.system_ready;
      amk_actual_values1.amk_status.error = can_amk.amk_status.error;
      amk_actual_values1.amk_status.warn = can_amk.amk_status.warn;
      amk_actual_values1.amk_status.quit_dc_on = can_amk.amk_status.quit_dc_on;
      amk_actual_values1.amk_status.dc_on = can_amk.amk_status.dc_on;
      amk_actual_values1.amk_status.quit_inverter_on = can_amk.amk_status.quit_inverter_on;
      amk_actual_values1.amk_status.inverter_on = can_amk.amk_status.inverter_on;
      amk_actual_values1.amk_status.derating = can_amk.amk_status.derating;
      amk_actual_values1.actual_velocity = can_amk.actual_velocity;
      amk_actual_values1.torque_current = can_amk.torque_current;
      amk_actual_values1.magnetizing_current = can_amk.magnetizing_current;
      amk_front_right_actual_values1_publisher->publish(amk_actual_values1);
      break;
    }

    case can_id<AmkFrontRightActualValues2>: {
      auto can_amk = convert<AmkFrontRightActualValues2>(frame);
      amk_actual_values2.temp_motor = can_amk.temp_motor;
      amk_actual_values2.temp_inverter = can_amk.temp_inverter;
      amk_actual_values2.error_info = can_amk.error_info;
      amk_actual_values2.temp_igbt = can_amk.temp_igbt;
      amk_front_right_actual_values2_publisher->publish(amk_actual_values2);
      break;
    }

    case can_id<AmkRearLeftActualValues1>: {
      auto can_amk = convert<AmkRearLeftActualValues1>(frame);
      amk_actual_values1.amk_status.system_ready = can_amk.amk_status.system_ready;
      amk_actual_values1.amk_status.error = can_amk.amk_status.error;
      amk_actual_values1.amk_status.warn = can_amk.amk_status.warn;
      amk_actual_values1.amk_status.quit_dc_on = can_amk.amk_status.quit_dc_on;
      amk_actual_values1.amk_status.dc_on = can_amk.amk_status.dc_on;
      amk_actual_values1.amk_status.quit_inverter_on = can_amk.amk_status.quit_inverter_on;
      amk_actual_values1.amk_status.inverter_on = can_amk.amk_status.inverter_on;
      amk_actual_values1.amk_status.derating = can_amk.amk_status.derating;
      amk_actual_values1.actual_velocity = can_amk.actual_velocity;
      amk_actual_values1.torque_current = can_amk.torque_current;
      amk_actual_values1.magnetizing_current = can_amk.magnetizing_current;
      amk_rear_left_actual_values1_publisher->publish(amk_actual_values1);
      break;
    }

    case can_id<AmkRearLeftActualValues2>: {
      auto can_amk = convert<AmkRearLeftActualValues2>(frame);
      amk_actual_values2.temp_motor = can_amk.temp_motor;
      amk_actual_values2.temp_inverter = can_amk.temp_inverter;
      amk_actual_values2.error_info = can_amk.error_info;
      amk_actual_values2.temp_igbt = can_amk.temp_igbt;
      amk_rear_left_actual_values2_publisher->publish(amk_actual_values2);
      break;
    }

    case can_id<AmkRearRightActualValues1>: {
      auto can_amk = convert<AmkRearRightActualValues1>(frame);
      amk_actual_values1.amk_status.system_ready = can_amk.amk_status.system_ready;
      amk_actual_values1.amk_status.error = can_amk.amk_status.error;
      amk_actual_values1.amk_status.warn = can_amk.amk_status.warn;
      amk_actual_values1.amk_status.quit_dc_on = can_amk.amk_status.quit_dc_on;
      amk_actual_values1.amk_status.dc_on = can_amk.amk_status.dc_on;
      amk_actual_values1.amk_status.quit_inverter_on = can_amk.amk_status.quit_inverter_on;
      amk_actual_values1.amk_status.inverter_on = can_amk.amk_status.inverter_on;
      amk_actual_values1.amk_status.derating = can_amk.amk_status.derating;
      amk_actual_values1.actual_velocity = can_amk.actual_velocity;
      amk_actual_values1.torque_current = can_amk.torque_current;
      amk_actual_values1.magnetizing_current = can_amk.magnetizing_current;
      amk_rear_right_actual_values1_publisher->publish(amk_actual_values1);
      break;
    }

    case can_id<AmkRearRightActualValues2>: {
      auto can_amk = convert<AmkRearRightActualValues2>(frame);
      amk_actual_values2.temp_motor = can_amk.temp_motor;
      amk_actual_values2.temp_inverter = can_amk.temp_inverter;
      amk_actual_values2.error_info = can_amk.error_info;
      amk_actual_values2.temp_igbt = can_amk.temp_igbt;
      amk_rear_right_actual_values2_publisher->publish(amk_actual_values2);
      break;
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}
