#define CATCH_CONFIG_MAIN

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <catch2/catch.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

/**
 * @brief Integration test for the Talker node in ROS2.
 *
 * This test suite checks the functionality of the Talker node by verifying
 * that:
 * 1. The publisher is correctly sending messages on the "/chatter" topic.
 * 2. The node is correctly broadcasting a transform from the "world" frame to
 * the "talk" frame.
 */
TEST_CASE("Talker node integration test", "[talker]") {
  // Create a test node for Talker node integration testing
  auto node = rclcpp::Node::make_shared("talker_test_node");

  /**
   * @brief Test case for checking the publisher on the "/chatter" topic.
   *
   * This test subscribes to the "/chatter" topic and verifies that the
   * published message contains the string "Current time".
   */
  SECTION("Publisher Test") {
    // Create a subscription to the "/chatter" topic
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      "/chatter", 10, [&](std_msgs::msg::String::SharedPtr msg) {
        // Check if the message data contains "Current time"
        REQUIRE(msg->data.find("Current time") != std::string::npos);
      });

    // Spin the node for a short period to allow message to be received
    rclcpp::spin_some(node);
  }

  /**
   * @brief Test case for checking the transform broadcast from the "talk" frame
   * to the "world" frame.
   *
   * This test verifies that the Talker node broadcasts a transform from the
   * "talk" frame to the "world" frame with the correct translation and rotation
   * values.
   */
  SECTION("TF Broadcast Test") {
    // Create a buffer and listener to listen for transforms
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Variable to store the result of the transform lookup
    bool transform_received = false;
    try {
      // Look up the transform from "world" to "talk"
      auto transform = tf_buffer.lookupTransform(
        "world", "talk", tf2::TimePointZero, tf2::Duration(1));
      transform_received = true;

      // Check the translation values
      REQUIRE(transform.transform.translation.x == 1.0);
      REQUIRE(transform.transform.translation.y == 1.0);
      REQUIRE(transform.transform.translation.z == 0.0);

      // Check the rotation values (set as 45 degrees around Z axis)
      REQUIRE(transform.transform.rotation.x == Approx(0.0).epsilon(0.01));
      REQUIRE(transform.transform.rotation.y == Approx(0.0).epsilon(0.01));
      REQUIRE(
        transform.transform.rotation.z ==
        Approx(0.7071).epsilon(0.01));        // sin(45 degrees) = 0.7071
      REQUIRE(
        transform.transform.rotation.w ==
        Approx(0.7071).epsilon(0.01));        // cos(45 degrees) = 0.7071
    } catch (tf2::TransformException & ex) {
      // If transform is not found, fail the test
      FAIL("Transform not found: " << ex.what());
    }

    // Verify that the transform was received
    REQUIRE(transform_received);
  }
}
