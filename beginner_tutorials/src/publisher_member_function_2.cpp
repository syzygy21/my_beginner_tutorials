/**
 * @file minimal_publisher.cpp
 * @brief Implementation of the MinimalPublisher class that publishes
 * customizable string messages
 * @author Navdeep
 * @copyright 2024
 *
 * This file implements a ROS2 publisher node that publishes string messages at
 * a configurable frequency. It also provides a service to modify the message
 * content dynamically.
 */

#include "beginner_tutorials/minimal_publisher.hpp"
using namespace std::chrono_literals;

/**
 * @brief Constructs a new MinimalPublisher object
 *
 * Initializes the publisher with configurable frequency and sets up the service
 * for changing message content. Includes comprehensive error checking and
 * logging.
 *
 * @throws std::runtime_error If publisher creation fails
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
 * @brief Timer callback function for publishing messages
 *
 * Publishes the current message text along with an incrementing counter.
 * Includes warning for high message counts.
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
  q.setRPY(0.0, 0.0, count_ * 0.1); // Rotation around Z-axis
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform);

}

/**
 * @brief Service callback to change the message text
 *
 * @param request Boolean request to change the message (true) or reset to
 * default (false)
 * @param response Contains success status and confirmation message
 *
 * Changes the message text based on the request value:
 * - true: Sets to extended introduction
 * - false: Resets to default message
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
 * @brief Main function to run the publisher node
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int 0 on successful execution, non-zero on error
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
