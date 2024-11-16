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

#include <chrono>  
#include <memory>  

#include <tf2/LinearMath/Quaternion.h>  
#include <tf2_ros/transform_broadcaster.h>  

#include <geometry_msgs/msg/transform_stamped.hpp> 
#include <rclcpp/rclcpp.hpp>  

/**
 * @brief A ROS2 node that broadcasts a transform between two frames: "world"
 * and "talk".
 *
 * This node periodically sends the transform between the "world" frame and the
 * "talk" frame. The transform includes both translation and rotation data.
 */
class Talker : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for Talker class, initializes the transform broadcaster
   *        and sets up a timer to broadcast transforms at regular intervals.
   */
  Talker() : Node("talker") {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create a timer to broadcast transform every 500 milliseconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&Talker::broadcast_tf, this));
  }

 private:
  /**
   * @brief Broadcasts a transform from the "world" frame to the "talk" frame.
   *
   * This function fills a TransformStamped message with translation and
   * rotation data, then sends it using the transform broadcaster.
   */
  void broadcast_tf() {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp =
        this->get_clock()->now();                ///< Timestamp of the transform
    transformStamped.header.frame_id = "world";  ///< Parent frame
    transformStamped.child_frame_id = "talk";    ///< Child frame

    // Translation data (in meters)
    transformStamped.transform.translation.x = 1.0;
    transformStamped.transform.translation.y = 1.0;
    transformStamped.transform.translation.z = 0.0;

    // Rotation data (in quaternion form)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.785);  ///< 45 degrees rotation around Z-axis
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // Send the transform
    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for broadcasting transform
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< Transform broadcaster
};

/**
 * @brief Main entry point for the ROS2 program.
 *
 * Initializes the ROS2 client library, starts the Talker node, and handles
 * shutdown.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);                  ///< Initialize ROS2 client library
  rclcpp::spin(std::make_shared<Talker>());  ///< Spin the Talker node to
                                             ///< process callbacks
  rclcpp::shutdown();                        ///< Shutdown ROS2 client library
  return 0;
}

