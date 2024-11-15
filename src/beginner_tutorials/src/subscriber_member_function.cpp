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

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief MinimalSubscriber class subscribes to the "custom_topic" topic to
 * receive time-based messages.
 *
 * This node subscribes to the "custom_topic" and listens for messages of type
 * std_msgs::msg::String. Depending on the content of the message, different log
 * levels are used to log the message. If the message contains certain keywords
 * (DEBUG, WARN, ERROR, or FATAL), the corresponding log level is used to print
 * the message, otherwise, it defaults to INFO.
 */
class MinimalSubscriber : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for MinimalSubscriber class.
   *
   * Initializes the subscriber to "custom_topic" and sets up the callback to
   * handle received messages.
   */
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // Initialize subscription to "custom_topic" with a queue size of 10.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "custom_topic", 10,
      std::bind(&MinimalSubscriber::TopicCallback, this, _1));
  }

private:
  /**
   * @brief Callback function that processes the received message.
   *
   * Logs the received message based on the content. If specific keywords such
   * as "DEBUG", "WARN", "ERROR", or "FATAL" are found in the message, the
   * corresponding log level is used.
   *
   * @param msg The received message.
   */
  void TopicCallback(const std_msgs::msg::String & msg) const
  {
    std::string received_message = msg.data;
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Received: '" << received_message << "'");

    // Use DEBUG level if message contains "DEBUG"
    if (received_message.find("DEBUG") != std::string::npos) {
      RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Debug level message detected in: '" << received_message << "'");
    } else if (received_message.find("WARN") != std::string::npos) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Warning level message detected in: '" << received_message << "'");
    } else if (received_message.find("ERROR") != std::string::npos) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Error level message detected in: '" << received_message << "'");
    } else if (received_message.find("FATAL") != std::string::npos) {
      RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Fatal level message detected in: '" << received_message << "'");
    } else {
      RCLCPP_INFO_STREAM(
        this->get_logger(), "General message received: '"
          << received_message << "'");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    subscription_;    ///< Subscription handle to receive messages.
};

/**
 * @brief Main function to initialize and run the MinimalSubscriber node.
 *
 * Initializes the ROS2 client library, creates a MinimalSubscriber node,
 * and enters the spinning loop to keep the node alive.
 *
 * @param argc Number of arguments passed to the program.
 * @param argv Array of arguments passed to the program.
 * @return int Exit status of the program.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
