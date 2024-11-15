/**
 * @file minimal_subscriber.cpp
 * @brief Implementation of a ROS2 subscriber node for string messages
 * @author Navdeep
 * @copyright 2024
 * @details This file implements a ROS2 subscriber node that:
 *   - Listens for string messages on the 'topic' topic
 *   - Performs message validation and error handling
 *   - Provides comprehensive logging at various severity levels
 */

#include "beginner_tutorials/minimal_subscriber.hpp"
using std::placeholders::_1;

/**
 * @brief Constructs a new MinimalSubscriber object
 * 
 * @details Initializes the node with the following components:
 *   - Creates a subscription to the 'topic' topic
 *   - Sets up message callback handling
 *   - Configures logging for node operations
 * 
 * @param None
 * @throws std::runtime_error If subscription creation fails
 * 
 * @note Uses a queue size of 10 for message buffering
 */
MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber") {
  // DEBUG - Log initialization details
  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Initializing subscriber node on topic 'topic'");
  try {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    // INFO - Log successful initialization
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Subscriber successfully initialized and listening on topic 'topic'");
  } catch (const std::exception& e) {
    // FATAL - Log critical initialization failure
    RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Failed to create subscription. Node cannot function: " << e.what());
    throw;  // Re-throw to terminate node
  }
}

/**
 * @brief Callback function for processing received messages
 * 
 * @param[in] msg The received string message to process
 * 
 * @details Performs the following validations and operations:
 *   - Checks for empty messages
 *   - Validates message length (warns if > 256 characters)
 *   - Processes and logs the received message content
 * 
 * @warning Logs warnings for empty messages and unusually long messages
 * @throws std::runtime_error If message processing fails
 * 
 * @note All logging is performed with appropriate severity levels
 */
void MinimalSubscriber::topic_callback(const std_msgs::msg::String& msg) const {
  // DEBUG - Log detailed message info
  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Received message of length: " << msg.data.length());
  // Validate message content
  if (msg.data.empty()) {
    // WARN - Log empty message warning
    RCLCPP_WARN_STREAM(this->get_logger(), "Received empty message");
    return;
  }
  // Check message length
  if (msg.data.length() > 256) {
    // WARN - Log unusually long message
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Received unusually long message: " << msg.data.length()
                           << " characters");
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

/**
 * @brief Main function to initialize and run the subscriber node
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Exit status
 *   @retval 0 Successful execution
 *   @retval 1 Error occurred during execution
 * 
 * @details Performs the following operations:
 *   - Initializes the ROS2 system
 *   - Creates and spins the subscriber node
 *   - Handles runtime errors with appropriate logging
 *   - Performs cleanup on shutdown
 */
int main(int argc, char* argv[]) {
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    // FATAL - Log critical runtime error
    auto logger = rclcpp::get_logger("minimal_subscriber");
    RCLCPP_FATAL_STREAM(
        logger, "Fatal error occurred, node shutting down: " << e.what());
    return 1;
  }
  return 0;
}
