/**
 * @file minimal_subscriber.hpp
 * @brief Header file for the MinimalSubscriber class
 * @author Navdeep
 * @copyright 2024
 *
 * This file declares a ROS2 subscriber node that listens to string messages
 * and provides comprehensive message validation and error handling.
 */

#ifndef MINIMAL_SUBSCRIBER_HPP_
#define MINIMAL_SUBSCRIBER_HPP_

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief A ROS2 node that subscribes to string messages
 *
 * This class implements a ROS2 subscriber that:
 * - Listens for string messages on a specified topic
 * - Validates incoming messages (empty checks, length validation)
 * - Provides comprehensive error handling and logging
 * - Processes and logs received messages
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   * Initializes the subscriber node and sets up the subscription
   * to the 'topic' topic with appropriate error handling and logging.
   *
   * @throws std::runtime_error If subscription creation fails
   */
  MinimalSubscriber();

 private:
  /**
   * @brief Callback function for processing received messages
   *
   * Processes incoming messages with various validation checks:
   * - Validates message content (empty check)
   * - Checks message length (warns if > 256 characters)
   * - Logs received messages at appropriate levels
   *
   * @param msg The received string message
   * @throws std::runtime_error If message processing fails
   */
  void topic_callback(const std_msgs::msg::String& msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  ///< String message subscriber
};

#endif  // MINIMAL_SUBSCRIBER_HPP_