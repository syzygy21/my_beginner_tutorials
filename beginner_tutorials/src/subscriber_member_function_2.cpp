#include "beginner_tutorials/minimal_subscriber.hpp"

using std::placeholders::_1;

MinimalSubscriber::MinimalSubscriber() 
    : Node("minimal_subscriber") 
{
    // DEBUG - Log initialization details
    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "Initializing subscriber node on topic 'topic'");
    
    try {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        
        // INFO - Log successful initialization
        RCLCPP_INFO_STREAM(this->get_logger(),
            "Subscriber successfully initialized and listening on topic 'topic'");
    } catch (const std::exception& e) {
        // FATAL - Log critical initialization failure
        RCLCPP_FATAL_STREAM(this->get_logger(),
            "Failed to create subscription. Node cannot function: " << e.what());
        throw; // Re-throw to terminate node
    }
}

void MinimalSubscriber::topic_callback(const std_msgs::msg::String& msg) const 
{
    // DEBUG - Log detailed message info
    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "Received message of length: " << msg.data.length());
    
    // Check for potential issues
    if (msg.data.empty()) {
        // WARN - Log empty message warning
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Received empty message");
        return;
    }
    
    if (msg.data.length() > 256) {
        // WARN - Log unusually long message
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Received unusually long message: " << msg.data.length() << " characters");
    }
    
    try {
        // INFO - Log received message
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data);
    } catch (const std::exception& e) {
        // ERROR - Log message processing error
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "Error processing received message: " << e.what());
    }
}

int main(int argc, char* argv[]) 
{
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalSubscriber>());
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        // FATAL - Log critical runtime error
        auto logger = rclcpp::get_logger("minimal_subscriber");
        RCLCPP_FATAL_STREAM(logger,
            "Fatal error occurred, node shutting down: " << e.what());
        return 1;
    }
    return 0;
}