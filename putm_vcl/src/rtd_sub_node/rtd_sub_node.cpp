#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
//#include "rtd_sub_node/rtd_sub_node.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<msg::Rtd>(
      "rtd", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const msg::Rtd::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: ''", msg->state);
    }
    rclcpp::Subscription<msg::Rtd>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}