/**
 * @file minimal_publisher.cpp
 * @brief Implementation of a ROS2 publisher node with dynamic message
 * modification
 * @author Navdeep
 * @copyright 2024
 * @details This file implements a ROS2 publisher node that:
 *   - Publishes customizable string messages at a configurable frequency
 *   - Broadcasts transforms between 'world' and 'talk' frames
 *   - Provides a service to modify message content dynamically
 */

#include "beginner_tutorials/minimal_publisher.hpp"
using namespace std::chrono_literals;

/**
 * @brief Constructs a new MinimalPublisher object
 *
 * @details Initializes the node with the following components:
 *   - A publisher for string messages
 *   - A transform broadcaster
 *   - A timer for periodic publishing
 *   - A service for modifying message content
 *
 * @param None
 * @throws std::runtime_error If publisher creation fails
 *
 * @note Default publishing frequency is 2.0 Hz and can be configured via ROS
 * parameter
 */
MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0), message_text_("I am Navdeep") {
  // Declare and validate the publishing frequency parameter
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->declare_parameter("publish_frequency", 2.0);
  double freq = this->get_parameter("publish_frequency").as_double();

  if (freq <= 0.0) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Invalid frequency value: "
                            << freq
                            << "Hz. Must be positive. Using default 2.0Hz");
    freq = 2.0;
  }

  // Calculate publishing period in milliseconds
  auto publish_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / freq));

  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Publisher initialized with frequency: "
                          << freq << "Hz (period: " << publish_period.count()
                          << "ms)");

  try {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  } catch (const std::exception& e) {
    RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Failed to create publisher. Node cannot function without publisher: "
            << e.what());
    throw;
  }

  // Create timer for periodic publishing
  timer_ = this->create_wall_timer(
      publish_period, std::bind(&MinimalPublisher::timer_callback, this));

  // Initialize service for changing message content
  service_ = this->create_service<example_interfaces::srv::SetBool>(
      "change_string",
      std::bind(&MinimalPublisher::change_string_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publisher initialized successfully on topic 'topic' with frequency "
          << freq << " Hz");
}

/**
 * @brief Timer callback function for publishing messages and transforms
 *
 * @details This callback:
 *   - Publishes the current message text with an incrementing counter
 *   - Broadcasts a transform from 'world' to 'talk' frame
 *   - Updates transform with rotation around Z-axis based on counter
 *
 * @warning Logs a warning when message count exceeds 10
 *
 * @note Transform parameters:
 *   - Translation: (1.0, 2.0, 0.5)
 *   - Rotation: Varies with counter (around Z-axis)
 */
void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = message_text_ + " " + std::to_string(count_++);

  // Warn if message count gets too high
  if (count_ > 10) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Message count is very high ("
                           << count_ << "). Consider resetting if needed.");
  }

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);

  // Broadcasting the transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "talk";

  // Define translation
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 0.5;

  // Define rotation using a quaternion
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, count_ * 0.1);  // Rotation around Z-axis
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform);
}

/**
 * @brief Service callback to modify the published message text
 *
 * @param[in] request Shared pointer to SetBool request
 *                    - true: Sets extended introduction
 *                    - false: Resets to default message
 * @param[out] response Shared pointer to SetBool response containing:
 *                     - success: Operation status
 *                     - message: Status description
 *
 * @details Changes the message text between two preset strings based on the
 * request:
 *   - Default: "I am Navdeep"
 *   - Extended: "My name is Navdeep and I am a robotics student"
 *
 * @note Logs error if null request is received
 */
void MinimalPublisher::change_string_callback(
    const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
  // Validate request
  if (!request) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Received null service request");
    return;
  }

  // Change message based on request
  if (request->data) {
    message_text_ = "My name is Navdeep and I am a robotics student";
    response->success = true;
    response->message = "Message changed successfully";
  } else {
    message_text_ = "I am Navdeep";
    response->success = true;
    response->message = "Message reset to default";
  }

  RCLCPP_INFO(this->get_logger(), "String changed to: %s",
              message_text_.c_str());
}

/**
 * @brief Main function to initialize and run the publisher node
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Exit status (0 for normal exit, non-zero for errors)
 *
 * @note Initializes ROS2, spins the node, and performs cleanup on shutdown
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
