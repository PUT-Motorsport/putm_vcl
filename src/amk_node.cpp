#include <cstdio>

#include <array>
#include <iostream>
#include <chrono>
#include <thread>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "amk_startup.hpp"
#include "putm_ev_amk_2023/amk_node.hpp"

using namespace std::chrono_literals;

using namespace PUTM_CAN;

using std::placeholders::_1;

uint8_t tourqe = 0;

// class MinimalSubscriber : public rclcpp::Node
// {
//   public:
//     MinimalSubscriber()
//     : Node("minimal_subscriber")
//     {
//       subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
//     }

//   private:
//     void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
//     {
//       tourqe = -1*(msg->axes[5]-1)*100;
//     }
//     rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
// };

// void send_frames()
// {
//   CanTx can_tx("can0");
//   while(true)
//   {
//     SetTorque(tourqe);
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }

// }

int main(int argc, char ** argv) {

  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  AMK_Startup();
  }

