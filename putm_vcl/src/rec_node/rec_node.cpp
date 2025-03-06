#include <rec_node/rec_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <putm_vcl_interfaces/msg/rtd.hpp>
#include <fstream>

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RecNode::RecNode()
    : Node("rec_node"),
    rtd_subscription(this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&RecNode::rtd_callback, this, _1))),
    recording(false) {}

void RecNode::rtd_callback(const msg::Rtd::SharedPtr msg) {
if (msg->state && !recording) {
    RCLCPP_INFO(this->get_logger(), "Car is ready. Starting data recording.");
    start_recording();
    if (stop_timer_)
    {
        stop_timer_->cancel();
        stop_timer_.reset();
    }
} else if (!msg->state && recording && !stop_timer_) {
        RCLCPP_INFO(this->get_logger(), "Car is not ready. Stopping data recording aftet 20 seconds.");
        stop_timer_ = this->create_wall_timer(
            20s,
            [this]() 
            {
                if (recording) {
                    RCLCPP_INFO(this->get_logger(), "20 seconds passed. Stopping recording.");
                    stop_recording();
                }
                stop_timer_.reset();
            }
        );
}
}

void RecNode::start_recording() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S");

    bag_file_name_ = "recording_" + oss.str(); //nazwa pliku zapisu

    RCLCPP_INFO(this->get_logger(), "Recording to: %s", bag_file_name_.c_str());

    start_command = "ros2 bag record -a -s mcap -o " + bag_file_name_ + " & echo $!";
    process_pid_ = std::system(start_command.c_str());
    RCLCPP_INFO(this->get_logger(), "Recording started with PID: %d", process_pid_);

    recording = true;
}

void RecNode::stop_recording()
{
    if (recording)
    {
        std::string kill_command = "kill " + std::to_string(process_pid_);
        std::system(stop_command.c_str());
        recording = false;
    }
    
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecNode>());
    rclcpp::shutdown();
    return 0;
}