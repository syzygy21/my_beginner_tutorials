#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using std_msgs::msg::String;

class PublisherTestFixture {
public:
    PublisherTestFixture() {
        test_node_ = rclcpp::Node::make_shared("publisher_test_node");
        
        test_node_->declare_parameter("test_duration", 10.0);
        test_node_->declare_parameter("expected_frequency", 2.0);
        
        test_duration_ = test_node_->get_parameter("test_duration").as_double();
        expected_frequency_ = test_node_->get_parameter("expected_frequency").as_double();
        
        message_count_ = 0;
        last_timestamp_ = std::chrono::steady_clock::now();
        
        subscription_ = test_node_->create_subscription<String>(
            "topic", 10,
            [this](const String::SharedPtr msg) {
                processMessage(msg);
            });
            
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(test_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

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

    bool verifyMessageFormat() const {
        return !received_messages_.empty() && 
               all_messages_valid_ && 
               message_count_ >= 3;  // Ensure we got enough messages
    }

    bool verifyFrequency() const {
        if (intervals_.size() < 2) return false;
        double avg_interval = std::accumulate(intervals_.begin(), 
                                            intervals_.end(), 0.0) / intervals_.size();
        double actual_freq = 1000.0 / avg_interval;  // Convert to Hz
        return std::abs(actual_freq - expected_frequency_) < 0.5;  // Allow 0.5 Hz tolerance
    }

    bool verifyTransform() {
        try {
            auto transform = tf_buffer_->lookupTransform("world", "talk", tf2::TimePointZero);
            return (std::abs(transform.transform.translation.x - 1.0) < 0.01 &&
                   std::abs(transform.transform.translation.y - 2.0) < 0.01 &&
                   std::abs(transform.transform.translation.z - 0.5) < 0.01);
        } catch (const tf2::TransformException& ex) {
            return false;
        }
    }

private:
    void processMessage(const String::SharedPtr msg) {
        received_messages_.push_back(msg->data);
        message_count_++;
        
        // Verify message format
        all_messages_valid_ &= (msg->data.find("I am Navdeep") == 0);
        
        // Calculate interval
        auto now = std::chrono::steady_clock::now();
        if (message_count_ > 1) {
            intervals_.push_back(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_timestamp_).count());
        }
        last_timestamp_ = now;
    }

    bool hasReceivedMessages() const {
        return message_count_ >= 3;  // Wait for at least 3 messages
    }

    rclcpp::Node::SharedPtr test_node_;
    rclcpp::Subscription<String>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::vector<std::string> received_messages_;
    std::vector<double> intervals_;
    bool all_messages_valid_ = true;
    int message_count_;
    std::chrono::steady_clock::time_point last_timestamp_;
    double test_duration_;
    double expected_frequency_;
};

TEST_CASE_METHOD(PublisherTestFixture, "Test MinimalPublisher Functionality", "[publisher]") {
    REQUIRE(waitForMessages());
    CHECK(verifyMessageFormat());
    CHECK(verifyFrequency());
    CHECK(verifyTransform());
}