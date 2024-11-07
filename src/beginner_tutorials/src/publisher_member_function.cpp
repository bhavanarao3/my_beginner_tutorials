// Copyright 2024 Bhavana B Rao bhavana3@umd.edu
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

#include <ctime>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/set_bool.hpp"  // Include SetBool service header

/**
 * @class MinimalPublisher
 * @brief A ROS2 node that publishes messages to a topic and provides a service to toggle the base output string.
 *
 * This class demonstrates a basic publisher that sends messages containing the current time to a topic
 * named `custom_topic`. It also provides a service to toggle the base output string, allowing dynamic
 * changes in the message format. The node also handles various logging levels based on user-defined parameters.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the MinimalPublisher node.
   *
   * Initializes the publisher, service, timer, and parameters for publish frequency, frequency threshold, 
   * and log level. The node's constructor sets up the necessary components for publishing messages and
   * handling service requests.
   */
  MinimalPublisher() : Node("minimal_publisher"), base_output_("Current time: "), publish_frequency_(500) {
    // Declare parameters for log level and frequency threshold
    this->declare_parameter<int>("publish_frequency", 500);
    this->declare_parameter<int>("frequency_threshold", 200);
    this->declare_parameter<int>("log_level", 1);  // Default to INFO log level

    // Initialize publisher on 'custom_topic' with a queue size of 10.
    publisher_ = this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    
    // Initialize service to modify the base output string.
    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "toggle_base_output",
        std::bind(&MinimalPublisher::ChangeBaseOutputCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Timer to trigger the publishing.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_frequency_),
        std::bind(&MinimalPublisher::TimerCallback, this));

    // Get parameter values
    this->get_parameter("publish_frequency", publish_frequency_);
    this->get_parameter("frequency_threshold", frequency_threshold_);
    this->get_parameter("log_level", log_level_);
  }

 private:
  /**
   * @brief Callback function for the timer to publish messages.
   *
   * This function is called periodically based on the specified publish frequency. It publishes the current
   * time to the topic `custom_topic`. It also handles various log levels and conditions like no active subscribers,
   * errors if the publish frequency is too low, and fatal conditions for very low publish frequencies.
   */
  void TimerCallback() {
    // Check the number of active subscribers before publishing
    if (publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN(this->get_logger(), "No active subscribers on 'custom_topic'.");
    }

    // Simulate log levels based on parameters
    if (publisher_->get_subscription_count() == 0) {
      if (log_level_ <= 2) {
        RCLCPP_WARN(this->get_logger(), "No subscribers, publishing to topic may not be useful.");
      }
    }

    auto message = std_msgs::msg::String();
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    message.data = base_output_ + std::string(std::ctime(&time));
    publisher_->publish(message);

    // Simulate error when publish frequency is below threshold
    if (publish_frequency_ < frequency_threshold_) {
      if (log_level_ <= 3) {
        RCLCPP_ERROR(this->get_logger(), "Publish frequency is below the threshold! Frequency: %d", publish_frequency_);
      }
    }

    // Simulate fatal condition if a critical condition is met (e.g., frequency is too low)
    if (publish_frequency_ < 100) {
      if (log_level_ <= 4) {
        RCLCPP_FATAL(this->get_logger(), "Fatal error: Publish frequency is too low! Terminating.");
        rclcpp::shutdown();  // Shutdown ROS2 on fatal error
      }
    }

    // Simulate INFO and DEBUG based on log level
    if (log_level_ <= 1) {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }
    if (log_level_ <= 0) {
      RCLCPP_DEBUG(this->get_logger(), "Debug: Message published at %s", message.data.c_str());
    }
  }

  /**
   * @brief Callback function to toggle the base output string via the service.
   *
   * This service allows clients to toggle the base output string that is used in the published messages.
   * If the request data is `true`, the base output is set to "Custom time: ". Otherwise, it is set to
   * "Current time: ".
   *
   * @param request The service request containing the new base output string flag.
   * @param response The service response confirming the operation's success.
   */
  void ChangeBaseOutputCallback(
      const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
      std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    if (request->data) {
      base_output_ = "Custom time: ";
    } else {
      base_output_ = "Current time: ";
    }
    response->success = true;
    if (log_level_ <= 1) {
      RCLCPP_INFO(this->get_logger(), "Base output string changed to: '%s'", base_output_.c_str());
    }
  }

  // Base output string for publishing
  std::string base_output_;

  // Timer that triggers the publisher
  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher handle
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // Service handle
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;

  // Publish frequency in milliseconds
  int publish_frequency_;

  // Frequency threshold for triggering an error
  int frequency_threshold_;

  // Log level to control verbosity (0: DEBUG, 1: INFO, 2: WARN, 3: ERROR, 4: FATAL)
  int log_level_;
};

/**
 * @brief Main function to initialize the ROS2 node and spin it.
 *
 * Initializes the ROS2 system and runs the `MinimalPublisher` node. The node will continue spinning until
 * it is manually shutdown.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The array of arguments passed to the program.
 * @return 0 on successful execution, otherwise non-zero.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
