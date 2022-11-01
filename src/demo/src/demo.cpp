#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TalkPublisher : public rclcpp::Node
{
public:
    TalkPublisher()
            : Node("talk_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("speech", 10);
        timer_ = this->create_wall_timer(
                500ms, std::bind(&TalkPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        time_t now;
        auto local_time = localtime(&now);
        auto hour = local_time->tm_hour;
        if(0 <= hour && hour < 12) {
            message.data = "Good morning! ";
        } else if(12 <= hour && hour < 18) {
            message.data = "Good afternoon! ";
        } else {
            message.data = "Good evening! ";
        }
        message.data += "Today is ";
        now = std::time(0);
        auto now_str = ctime(&now);
        message.data += now_str;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        rclcpp::shutdown();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkPublisher>());
    rclcpp::shutdown();
    return 0;
}
