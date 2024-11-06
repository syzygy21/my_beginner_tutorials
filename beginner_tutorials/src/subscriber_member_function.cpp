// Copyright 2024 Navdeep Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** @file subscriber.cpp
 *  @brief A minimal ROS2 subscriber example.
 *
 *  This file implements a basic ROS2 subscriber that listens for string
 *  messages on a topic and processes them.
 *
 *  @author Navdeep Singh
 *  @date November 6, 2024
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief A minimal subscriber node class that demonstrates basic ROS2
 * subscription.
 *
 * This class creates a ROS2 node that subscribes to string messages on a topic
 * and processes them using a callback function.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalSubscriber
   *
   * Initializes the subscriber node and creates a subscription to the 'topic'
   * topic. Sets up the callback function for processing received messages.
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function for processing received messages
   *
   * This function is called whenever a message is received on the subscribed
   * topic. It processes the received message and logs it using ROS2 logging.
   *
   * @param msg The received string message
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  /**
   * @brief Subscription handle for receiving messages
   *
   * Shared pointer to the subscription object that manages the message
   * subscription.
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main function for the subscriber node
 *
 * Initializes ROS2, creates and spins the subscriber node, and handles
 * shutdown.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Return value indicating success (0) or failure (non-zero)
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

