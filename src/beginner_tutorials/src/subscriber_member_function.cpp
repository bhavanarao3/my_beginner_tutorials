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

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>  // For std::shared_ptr
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief A ROS2 node that listens to a transform between the "world" and "talk"
 * frames.
 *
 * This node subscribes to the transform between the "world" frame and the
 * "talk" frame, retrieves the transform, and logs the translation and rotation
 * data.
 */
class Listener : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for Listener class, initializes the transform listener
   *        and sets up a timer to periodically check for transforms.
   */
  Listener() : Node("listener") {
    // Initialize the transform buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer to check for transform every second
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&Listener::lookup_transform, this));
  }

 private:
  /**
   * @brief Look up the transform from "world" to "talk" frame and logs
   * translation and rotation.
   *
   * This function retrieves the transform and logs the position and orientation
   * of the "talk" frame relative to the "world" frame.
   */
  void lookup_transform() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      // Look up the transform from "world" frame to "talk" frame
      transformStamped =
          tf_buffer_->lookupTransform("world", "talk", tf2::TimePointZero);

      // Log translation and rotation data
      RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f, z=%f",
                  transformStamped.transform.translation.x,
                  transformStamped.transform.translation.y,
                  transformStamped.transform.translation.z);

      RCLCPP_INFO(this->get_logger(), "Rotation: x=%f, y=%f, z=%f, w=%f",
                  transformStamped.transform.rotation.x,
                  transformStamped.transform.rotation.y,
                  transformStamped.transform.rotation.z,
                  transformStamped.transform.rotation.w);
    } catch (const tf2::TransformException& ex) {
      // If transform is not available, log the error
      RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s",
                   ex.what());
    }
  }

  rclcpp::TimerBase::SharedPtr
      timer_;  ///< Timer for periodically checking for transform
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_;                             ///< Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< Transform buffer
};

/**
 * @brief Main entry point for the ROS2 program.
 *
 * Initializes the ROS2 client library, starts the Listener node, and handles
 * shutdown.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  ///< Initialize ROS2 client library
  rclcpp::spin(std::make_shared<Listener>());  ///< Spin the Listener node to
                                               ///< process callbacks
  rclcpp::shutdown();                          ///< Shutdown ROS2 client library
  return 0;
}
