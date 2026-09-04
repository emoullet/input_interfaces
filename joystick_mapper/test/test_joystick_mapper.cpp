#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "extender_msgs/msg/cartesian_velocity_command.hpp"
#include "joystick_mapper/joystick_mapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{
  using namespace std::chrono_literals;

  class JoystickMapperTest : public ::testing::Test
  {
  protected:
    static void SetUpTestSuite()
    {
      setenv("ROS_LOG_DIR", "/tmp", 1);
      rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
      rclcpp::shutdown();
    }
  };
} // namespace

TEST_F(JoystickMapperTest, PublishesConfiguredOrientationFrame)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("joy_topic", "/test_orientation_frame/joy"),
       rclcpp::Parameter("output_topic", "/test_orientation_frame/command"),
       rclcpp::Parameter("mode_request_topic", "/test_orientation_frame/mode"),
       rclcpp::Parameter("output_frame_id", "base_link"),
       rclcpp::Parameter("orientation_frame_id", "hybrid_frame"),
       rclcpp::Parameter("deadzone", 0.0),
       rclcpp::Parameter("modes.names", std::vector<std::string>{"b1"}),
       rclcpp::Parameter("modes.b1.axes.linear_x.index", 0),
       rclcpp::Parameter("modes.b1.axes.linear_y.index", 1),
       rclcpp::Parameter("modes.b1.axes.linear_z.index", 2),
       rclcpp::Parameter("modes.b1.axes.angular_x.index", 3),
       rclcpp::Parameter("modes.b1.axes.angular_y.index", 4),
       rclcpp::Parameter("modes.b1.axes.angular_z.index", 5)});
  auto mapper = std::make_shared<joystick_mapper::JoystickMapper>(options);
  auto test_node = std::make_shared<rclcpp::Node>("joystick_mapper_test_client");

  std::optional<extender_msgs::msg::CartesianVelocityCommand> received;
  auto command_sub = test_node->create_subscription<extender_msgs::msg::CartesianVelocityCommand>(
      "/test_orientation_frame/command", 10,
      [&received](const extender_msgs::msg::CartesianVelocityCommand &msg) { received = msg; });
  auto joy_pub =
      test_node->create_publisher<sensor_msgs::msg::Joy>("/test_orientation_frame/joy", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mapper);
  executor.add_node(test_node);

  sensor_msgs::msg::Joy joy;
  joy.axes = {0.1F, -0.2F, 0.3F, -0.4F, 0.5F, -0.6F};
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline)
  {
    joy_pub->publish(joy);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(received->header.frame_id, "base_link");
  EXPECT_EQ(received->orientation_frame_id, "hybrid_frame");
  EXPECT_NEAR(received->twist.linear.x, 0.1, 1e-6);
  EXPECT_NEAR(received->twist.linear.y, -0.2, 1e-6);
  EXPECT_NEAR(received->twist.linear.z, 0.3, 1e-6);
  EXPECT_NEAR(received->twist.angular.x, -0.4, 1e-6);
  EXPECT_NEAR(received->twist.angular.y, 0.5, 1e-6);
  EXPECT_NEAR(received->twist.angular.z, -0.6, 1e-6);
  (void)command_sub;
}

TEST_F(JoystickMapperTest, DefaultsToBaseFrame)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("joy_topic", "/test_default_frame/joy"),
       rclcpp::Parameter("output_topic", "/test_default_frame/command"),
       rclcpp::Parameter("mode_request_topic", "/test_default_frame/mode")});
  auto mapper = std::make_shared<joystick_mapper::JoystickMapper>(options);
  EXPECT_EQ(mapper->get_parameter("orientation_frame_id").as_string(), "base_frame");
}

TEST_F(JoystickMapperTest, RejectsInvalidOrientationFrame)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("orientation_frame_id", "ee_frame")});
  EXPECT_THROW(std::make_shared<joystick_mapper::JoystickMapper>(options), std::invalid_argument);
}

TEST_F(JoystickMapperTest, CyclesThroughConfiguredModesAndWrapsAround)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("joy_topic", "/test_mode_cycle/joy"),
       rclcpp::Parameter("output_topic", "/test_mode_cycle/command"),
       rclcpp::Parameter("mode_request_topic", "/test_mode_cycle/mode"),
       rclcpp::Parameter("deadzone", 0.0),
       rclcpp::Parameter("local_mode_button_index", 0),
       rclcpp::Parameter("local_mode_button_mode", "toggle"),
       rclcpp::Parameter("modes.names", std::vector<std::string>{"b1", "b2", "precision"}),
       rclcpp::Parameter("modes.b1.axes.linear_x.index", 0),
       rclcpp::Parameter("modes.b2.axes.linear_x.index", 1),
       rclcpp::Parameter("modes.precision.axes.linear_x.index", 2)});
  auto mapper = std::make_shared<joystick_mapper::JoystickMapper>(options);
  auto test_node = std::make_shared<rclcpp::Node>("joystick_mapper_mode_cycle_test_client");

  std::optional<extender_msgs::msg::CartesianVelocityCommand> received;
  auto command_sub = test_node->create_subscription<extender_msgs::msg::CartesianVelocityCommand>(
      "/test_mode_cycle/command", 10,
      [&received](const extender_msgs::msg::CartesianVelocityCommand &msg) { received = msg; });
  auto joy_pub = test_node->create_publisher<sensor_msgs::msg::Joy>("/test_mode_cycle/joy", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mapper);
  executor.add_node(test_node);

  auto publishUntilLinearX = [&](const sensor_msgs::msg::Joy &joy, double expected_linear_x) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    do
    {
      joy_pub->publish(joy);
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline &&
             (!received || std::abs(received->twist.linear.x - expected_linear_x) > 1e-6));
    ASSERT_TRUE(received.has_value());
    EXPECT_NEAR(received->twist.linear.x, expected_linear_x, 1e-6);
  };

  sensor_msgs::msg::Joy joy;
  joy.axes = {0.4F, 0.6F, 0.8F};
  joy.buttons = {0};

  // starts in modes.names[0] ("b1")
  publishUntilLinearX(joy, 0.4);

  // rising edge on local_mode_button_index cycles b1 -> b2
  joy.buttons = {1};
  publishUntilLinearX(joy, 0.6);

  // release then press again cycles b2 -> precision
  joy.buttons = {0};
  publishUntilLinearX(joy, 0.6);
  joy.buttons = {1};
  publishUntilLinearX(joy, 0.8);

  // release then press again wraps precision -> b1
  joy.buttons = {0};
  publishUntilLinearX(joy, 0.8);
  joy.buttons = {1};
  publishUntilLinearX(joy, 0.4);

  (void)command_sub;
}
