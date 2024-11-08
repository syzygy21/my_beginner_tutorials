/**
 * @file minimal_publisher.hpp
 * @brief Header file for the MinimalPublisher class
 * @author Navdeep
 * @copyright 2024
 * 
 * This file declares a ROS2 publisher node that publishes customizable string messages
 * at a configurable frequency and provides a service to modify the message content.
 */

#ifndef MINIMAL_PUBLISHER_HPP_
#define MINIMAL_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/set_bool.hpp"

/**
 * @brief A ROS2 node that publishes customizable string messages
 * 
 * This class implements a ROS2 publisher that:
 * - Publishes string messages at a configurable frequency
 * - Provides a service to modify the message content
 * - Includes comprehensive error checking and logging
 * - Maintains a message counter and customizable message text
 */
class MinimalPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Minimal Publisher object
     * 
     * Initializes the publisher with configurable frequency parameter,
     * sets up timer for periodic publishing, and creates a service
     * for dynamic message content modification.
     * 
     * @throws std::runtime_error If publisher or service creation fails
     */
    MinimalPublisher();

private:
    /**
     * @brief Timer callback for periodic message publishing
     * 
     * Publishes the current message text along with an incrementing counter.
     * Includes warning logging for high message counts.
     */
    void timer_callback();

    /**
     * @brief Service callback to modify the message content
     * 
     * @param request Shared pointer to the service request containing a boolean flag
     * @param response Shared pointer to the service response for success status
     * 
     * Changes the message text based on the request:
     * - true: Sets to extended introduction
     * - false: Resets to default message
     */
    void change_string_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response);

    rclcpp::TimerBase::SharedPtr timer_;        ///< Timer for periodic publishing
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< String message publisher
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;  ///< Service to modify message
    size_t count_;                              ///< Message counter
    std::string message_text_;                  ///< Current message content
};

#endif // MINIMAL_PUBLISHER_HPP_
