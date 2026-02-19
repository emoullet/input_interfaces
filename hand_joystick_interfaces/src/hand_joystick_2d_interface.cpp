#include "2D_hand_joystick_interface/hand_joystick_interface.hpp"
#include <cmath>

namespace hand_joystick_interface
{

HandJoystickInterface::HandJoystickInterface()
  : Node("hand_joystick_interface"),
    reference_set_(false)
{
  // Declare parameters
  this->declare_parameter("dead_zone", 0.05);
  this->declare_parameter("saturation_zone", 0.2);
  this->declare_parameter("landmark_index", 8);  // Index finger tip by default
  this->declare_parameter("output_round_decimals", 2);

  // Get parameters
  dead_zone_ = this->get_parameter("dead_zone").as_double();
  saturation_zone_ = this->get_parameter("saturation_zone").as_double();
  landmark_index_ = this->get_parameter("landmark_index").as_int();
  output_round_decimals_ = this->get_parameter("output_round_decimals").as_int();

  // Create subscriber for hand landmarks
  hand_landmarks_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
    "hand_landmarks",
    10,
    std::bind(&HandJoystickInterface::handLandmarksCallback, this, std::placeholders::_1)
  );

  // Create publisher for velocity commands
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("teleop_cmd", 10);

  RCLCPP_INFO(this->get_logger(), "Hand Joystick Interface initialized");
  RCLCPP_INFO(this->get_logger(), "Dead zone: %.2f", dead_zone_);
  RCLCPP_INFO(this->get_logger(), "Saturation zone: %.2f", saturation_zone_);
  RCLCPP_INFO(this->get_logger(), "Tracking landmark index: %d", landmark_index_);
  RCLCPP_INFO(this->get_logger(), "Output rounding decimals: %d", output_round_decimals_);
}

void HandJoystickInterface::handLandmarksCallback(
  const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
  // Check if we have enough landmarks (MediaPipe hand has 21 landmarks)
  if (msg->points.empty() || static_cast<int>(msg->points.size()) <= landmark_index_) {
    RCLCPP_WARN(this->get_logger(), "Received insufficient hand landmark data (got %zu points, need index %d)",
                msg->points.size(), landmark_index_);
    return;
  }

  // Extract x, y coordinates from the specified landmark
  // MediaPipe normalizes coordinates to [0, 1]
  double x = msg->points[landmark_index_].x;
  double y = msg->points[landmark_index_].y;

  // Set reference position on first callback
  if (!reference_set_) {
    reference_position_.x = x;
    reference_position_.y = y;
    reference_set_ = true;
    RCLCPP_INFO(this->get_logger(), "Reference position set: (%.3f, %.3f)", x, y);
    return;
  }

  // Calculate relative position from reference
  double dx = x - reference_position_.x;
  double dy = y - reference_position_.y;

  // Check dead zone
  if (isInDeadZone(dx, dy)) {
    // Publish zero velocity in dead zone
    auto twist_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist_msg);
    return;
  }

  // Calculate and publish velocity command
  auto twist_msg = calculateVelocityCommand(dx, dy);
  cmd_vel_pub_->publish(twist_msg);
}

geometry_msgs::msg::Twist HandJoystickInterface::calculateVelocityCommand(double x, double y)
{
  auto twist_msg = geometry_msgs::msg::Twist();

  // Normalize each axis independently between -1 and 1
  // Output saturates at ±1 when distance exceeds saturation_zone
  auto normalize_axis = [this](double value) -> double {
    if (std::abs(value) >= saturation_zone_) {
      return (value > 0.0) ? 1.0 : -1.0;
    } else {
      return value / saturation_zone_;
    }
  };

  auto round_value = [this](double value) -> double {
    double factor = std::pow(10.0, static_cast<double>(output_round_decimals_));
    return std::round(value * factor) / factor;
  };

  // Normalized output between -1 and 1
  twist_msg.linear.x = round_value(normalize_axis(x));
  twist_msg.linear.y = round_value(normalize_axis(y));
  twist_msg.linear.z = 0.0;

  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  return twist_msg;
}

bool HandJoystickInterface::isInDeadZone(double x, double y)
{
  double distance = std::sqrt(x * x + y * y);
  return distance < dead_zone_;
}

} // namespace hand_joystick_interface
