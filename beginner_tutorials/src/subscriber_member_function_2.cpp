/**
 * @file minimal_subscriber.cpp
 * @brief Implementation of the MinimalSubscriber class that subscribes to
 * string messages
 * @author Navdeep
 * @copyright 2024
 *
 * This file implements a ROS2 subscriber node that listens for string messages.
 * It includes comprehensive error handling and message validation.
 */

#include "beginner_tutorials/minimal_subscriber.hpp"
using std::placeholders::_1;

/**
 * @brief Constructs a new MinimalSubscriber object
 *
 * Initializes the subscriber node and sets up the subscription to the 'topic'
 * topic. Includes error handling and logging for initialization failures.
 *
 * @throws std::runtime_error If subscription creation fails
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
 * @brief Callback function that processes received messages
 *
 * Processes incoming string messages with various validation checks:
 * - Checks for empty messages
 * - Validates message length
 * - Handles message processing errors
 *
 * @param msg The received string message
 * @throws std::runtime_error If message processing fails
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
 * @brief Main function to run the subscriber node
 *
 * Initializes ROS2, creates and spins the subscriber node, and handles any
 * runtime errors.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int 0 on successful execution, 1 on error
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
