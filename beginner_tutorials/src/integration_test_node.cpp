/**
 * @file test_publisher.cpp
 * @brief Unit tests for the MinimalPublisher node using Catch2 framework
 * @author Navdeep
 * @copyright 2024
 * @details This file implements unit tests that verify:
 *   - Message publishing functionality
 *   - Message format validation
 *   - Publishing frequency accuracy
 *   - Transform broadcasting correctness
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

/**
 * @brief Test fixture class for MinimalPublisher testing
 *
 * @details Provides functionality to:
 *   - Set up a test node and subscription
 *   - Monitor and validate published messages
 *   - Verify transform broadcasts
 *   - Check publishing frequency
 */
class PublisherTestFixture {
 public:
  /**
   * @brief Construct a new Publisher Test Fixture object
   *
   * @details Initializes:
   *   - Test node with configurable parameters
   *   - Message subscription
   *   - TF buffer and listener
   *   - Message counting and timing mechanisms
   */
  PublisherTestFixture()
      : message_count_(0), test_duration_(10.0), expected_frequency_(2.0) {
    test_node_ = rclcpp::Node::make_shared("publisher_test_node");

    test_node_->declare_parameter("test_duration", test_duration_);
    test_node_->declare_parameter("expected_frequency", expected_frequency_);

    test_duration_ = test_node_->get_parameter("test_duration").as_double();
    expected_frequency_ =
        test_node_->get_parameter("expected_frequency").as_double();

    last_timestamp_ = std::chrono::steady_clock::now();

    subscription_ = test_node_->create_subscription<String>(
        "topic", 10,
        [this](const String::SharedPtr msg) { processMessage(msg); });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(test_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /**
   * @brief Waits for a specified number of messages to be received
   *
   * @param timeout Maximum time to wait for messages (default 5 seconds)
   * @return bool True if required messages were received, False if timeout
   * occurred
   *
   * @note Requires at least 3 messages for successful verification
   */
  bool waitForMessages(const std::chrono::seconds timeout = 5s) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && !hasReceivedMessages()) {
      rclcpp::spin_some(test_node_);
      rate.sleep();
      if ((std::chrono::steady_clock::now() - start) > timeout) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Verifies the format of received messages
   *
   * @return bool True if all messages are valid and minimum count is reached
   *
   * @details Checks:
   *   - Messages are not empty
   *   - All messages follow expected format
   *   - At least 3 messages were received
   */
  bool verifyMessageFormat() const {
    return !received_messages_.empty() && all_messages_valid_ &&
           message_count_ >= 3;  // Ensure we got enough messages
  }

  /**
   * @brief Verifies if publisher maintains expected frequency
   *
   * @return bool True if average publishing frequency matches expected
   * frequency
   *
   * @details Calculates average interval between messages and compares
   * with expected frequency (allows 0.5 Hz tolerance)
   */
  bool verifyFrequency() const {
    if (intervals_.size() < 2) return false;
    double avg_interval =
        std::accumulate(intervals_.begin(), intervals_.end(), 0.0) /
        intervals_.size();
    double actual_freq = 1000.0 / avg_interval;  // Convert to Hz
    return std::abs(actual_freq - expected_frequency_) <
           0.5;  // Allow 0.5 Hz tolerance
  }

  /**
   * @brief Verifies if transform is being broadcast correctly
   *
   * @return bool True if transform parameters match expected values
   *
   * @details Checks:
   *   - Transform from 'world' to 'talk' frame exists
   *   - Translation components match expected values (1.0, 2.0, 0.5)
   *   - Allows 0.01 tolerance for floating-point comparisons
   */
  bool verifyTransform() {
    try {
      auto transform =
          tf_buffer_->lookupTransform("world", "talk", tf2::TimePointZero);
      return (std::abs(transform.transform.translation.x - 1.0) < 0.01 &&
              std::abs(transform.transform.translation.y - 2.0) < 0.01 &&
              std::abs(transform.transform.translation.z - 0.5) < 0.01);
    } catch (const tf2::TransformException& ex) {
      return false;
    }
  }

 private:
  /**
   * @brief Processes received messages and performs validation
   *
   * @param msg Shared pointer to received string message
   *
   * @details Performs:
   *   - Message storage and counting
   *   - Format validation
   *   - Timing interval calculations
   */
  void processMessage(const String::SharedPtr msg) {
    received_messages_.push_back(msg->data);
    message_count_++;

    // Verify message format
    all_messages_valid_ =
        all_messages_valid_ && (msg->data.find("I am Navdeep") == 0);

    // Calculate interval
    auto now = std::chrono::steady_clock::now();
    if (message_count_ > 1) {
      intervals_.push_back(
          std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                last_timestamp_)
              .count());
    }
    last_timestamp_ = now;
  }

  /**
   * @brief Checks if minimum required messages have been received
   *
   * @return bool True if at least 3 messages have been received
   */
  bool hasReceivedMessages() const { return message_count_ >= 3; }

  rclcpp::Node::SharedPtr test_node_;  ///< Test node instance
  rclcpp::Subscription<String>::SharedPtr
      subscription_;  ///< Message subscription
  std::unique_ptr<tf2_ros::Buffer>
      tf_buffer_;  ///< TF buffer for transform operations
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_;  ///< TF listener instance

  std::vector<std::string>
      received_messages_;           ///< Storage for received messages
  std::vector<double> intervals_;   ///< Storage for message intervals
  bool all_messages_valid_ = true;  ///< Flag for message validation status
  int message_count_;               ///< Counter for received messages
  std::chrono::steady_clock::time_point
      last_timestamp_;         ///< Timestamp of last message
  double test_duration_;       ///< Duration for test execution
  double expected_frequency_;  ///< Expected publishing frequency
};

/**
 * @brief Test case for verifying MinimalPublisher functionality
 *
 * @details Tests:
 *   - Message reception within timeout
 *   - Message format correctness
 *   - Publishing frequency accuracy
 *   - Transform broadcast correctness
 *
 * @test Verifies complete functionality of MinimalPublisher node
 */
TEST_CASE_METHOD(PublisherTestFixture, "Test MinimalPublisher Functionality",
                 "[publisher]") {
  REQUIRE(waitForMessages());
  CHECK(verifyMessageFormat());
  CHECK(verifyFrequency());
  CHECK(verifyTransform());
}
