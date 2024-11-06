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

/** @file publisher.cpp
 *  @brief A minimal ROS2 publisher example.
 *
 *  This file implements a basic ROS2 publisher that periodically publishes
 *  string messages to a topic.
 *
 *  @author Navdeep Singh
 *  @date November 6, 2024
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief A minimal publisher node class that demonstrates basic ROS2
 * publishing.
 *
 * This class creates a ROS2 node that publishes string messages to a topic
 * at regular intervals using a timer.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher
   *
   * Initializes the publisher node, creates a publisher, and sets up a timer
   * for periodic message publishing.
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback function for publishing messages
   *
   * This function is called periodically by the timer. It creates and publishes
   * a string message containing a counter value.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "I am Navdeep. " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_; /**< Timer for periodic publishing */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_; /**< Publisher handle */
  size_t count_;  /**< Counter for message numbering */
};

/**
 * @brief Main function for the publisher node
 *
 * Initializes ROS2, creates and spins the publisher node, and handles shutdown.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Return value indicating success (0) or failure (non-zero)
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

