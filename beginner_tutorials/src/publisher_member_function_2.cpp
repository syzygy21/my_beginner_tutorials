#include "beginner_tutorials/minimal_publisher.hpp"

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0), message_text_("I am Navdeep")
{
    this->declare_parameter("publish_frequency", 2.0);
    double freq = this->get_parameter("publish_frequency").as_double();

    if (freq <= 0.0) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "Invalid frequency value: " << freq << "Hz. Must be positive. Using default 2.0Hz");
        freq = 2.0;
    }

    auto publish_period = std::chrono::milliseconds(static_cast<int>(1000.0 / freq));

    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "Publisher initialized with frequency: " << freq << "Hz (period: "
        << publish_period.count() << "ms)");

    try {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    } catch (const std::exception& e) {
        RCLCPP_FATAL_STREAM(this->get_logger(),
            "Failed to create publisher. Node cannot function without publisher: "
            << e.what());
        throw;
    }

    timer_ = this->create_wall_timer(
        publish_period, std::bind(&MinimalPublisher::timer_callback, this));

    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "change_string",
        std::bind(&MinimalPublisher::change_string_callback, this,
            std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(),
        "Publisher initialized successfully on topic 'topic' with frequency " << freq << " Hz");
}

void MinimalPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = message_text_ + " " + std::to_string(count_++);

    if (count_ > 10) {
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Message count is very high (" << count_ << "). Consider resetting if needed.");
    }

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void MinimalPublisher::change_string_callback(
    const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
{
    if (!request) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "Received null service request");
        return;
    }

    if (request->data) {
        message_text_ = "My name is Navdeep and I am a robotics student";
        response->success = true;
        response->message = "Message changed successfully";
    } else {
        message_text_ = "I am Navdeep";
        response->success = true;
        response->message = "Message reset to default";
    }

    RCLCPP_INFO(this->get_logger(), "String changed to: %s", message_text_.c_str());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}