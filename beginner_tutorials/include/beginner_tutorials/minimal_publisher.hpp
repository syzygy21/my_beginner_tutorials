#ifndef MINIMAL_PUBLISHER_HPP_
#define MINIMAL_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher();

private:
    void timer_callback();
    void change_string_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
    size_t count_;
    std::string message_text_;
};

#endif  // MINIMAL_PUBLISHER_HPP_