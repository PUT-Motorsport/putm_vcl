#include <rec_node/rec_node.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RecNode::RecNode()
    : Node("rec_node"),
    rtd_subscription(this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&RecNode::rtd_callback, this, _1))),
    recording(false),pid_(-1) {}

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
        RCLCPP_INFO(this->get_logger(), "Car is not ready. Stopping data recording after 20 seconds.");
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
    oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S")
    ;
    std::string filename = "/home/putm/rosbag/recording_" + oss.str();
    
    pid_ = fork();
    if (pid_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Błąd podczas tworzenia procesu!");
        return;
    }

    if (pid_ == 0) {
        // Proces potomny - wykonuje `ros2 bag record -a`
        execlp("ros2", "ros2", "bag", "record", "-a", "-s", "mcap", "-o", filename.c_str(), nullptr);
        // Jeśli execlp się nie powiodło
        RCLCPP_ERROR(this->get_logger(), "Nie udało się uruchomić `ros2 bag record`!");
        _exit(1);
    } else {
        // Proces macierzysty - zapamiętuje PID i kontynuuje
        RCLCPP_INFO(this->get_logger(), "Rozpoczęto nagrywanie. PID: %d", pid_);
        recording = true;
    }

}

void RecNode::stop_recording() {
    if (recording) {
        if (pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Zatrzymuję proces nagrywania (PID: %d)...", pid_);
            kill(pid_, SIGTERM);

            // Oczekiwanie na zakończenie procesu
            int status;
            waitpid(pid_, &status, 0);
            if (WIFEXITED(status)) {
                RCLCPP_INFO(this->get_logger(), "Proces ros2 bag record zakończony poprawnie.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Proces ros2 bag record zakończył się w sposób nieoczekiwany.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Niepoprawny PID procesu ros2 bag record.");
        }
        recording = false;
    }
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecNode>());
    rclcpp::shutdown();
    return 0;
}