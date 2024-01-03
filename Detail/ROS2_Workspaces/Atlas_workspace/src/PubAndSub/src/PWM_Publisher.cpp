#include <chrono>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class PWMPublisher : public rclcpp::Node {
public:
    PWMPublisher() : Node("pwm_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("pwm_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                                         std::bind(&PWMPublisher::publishPWM, this));
    }

private:
    void publishPWM() {
        auto message = std::make_unique<std_msgs::msg::Int32>();
        message->data = rand() % 101; // Generate a random number from 0 to 100
        publisher_->publish(std::move(message));
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PWMPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
