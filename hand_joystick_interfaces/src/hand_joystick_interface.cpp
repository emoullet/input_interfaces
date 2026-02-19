#include "hand_joystick_interface/hand_joystick_interface.hpp"
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace hand_joystick_interface
{

HandJoystickInterface::HandJoystickInterface()
  : Node("hand_joystick_interface"),
    reference_set_(false),
    filter_linear_x_(30.0, 1.0, 0.1),
    filter_linear_y_(30.0, 1.0, 0.1),
    filter_linear_z_(30.0, 1.0, 0.1)
{
  // Declare parameters
  this->declare_parameter("dead_zone", 0.05);
  this->declare_parameter("saturation_zone", 0.2);
  this->declare_parameter("depth_saturation_zone", 0.3);
  this->declare_parameter("landmark_index", 8);  // Index finger tip by default
  this->declare_parameter("output_round_decimals", 2);
  this->declare_parameter("enable_marker_visualization", true);
  this->declare_parameter("teleop_cmd_marker_topic", "teleop_cmd_marker");
  this->declare_parameter("teleop_cmd_marker_frame_id", "base_link");
  this->declare_parameter("teleop_cmd_marker_scale_x", 0.03);
  this->declare_parameter("teleop_cmd_marker_scale_y", 0.07);
  this->declare_parameter("teleop_cmd_marker_scale_z", 0.07);
  this->declare_parameter("filter_frequency", 30.0);
  this->declare_parameter("filter_mincutoff", 1.0);
  this->declare_parameter("filter_beta", 0.1);

  // Get parameters
  dead_zone_ = this->get_parameter("dead_zone").as_double();
  saturation_zone_ = this->get_parameter("saturation_zone").as_double();
  depth_saturation_zone_ = this->get_parameter("depth_saturation_zone").as_double();
  landmark_index_ = this->get_parameter("landmark_index").as_int();
  output_round_decimals_ = this->get_parameter("output_round_decimals").as_int();
  enable_marker_visualization_ = this->get_parameter("enable_marker_visualization").as_bool();
  const auto cmd_vel_marker_topic = this->get_parameter("teleop_cmd_marker_topic").as_string();
  marker_frame_id_ = this->get_parameter("teleop_cmd_marker_frame_id").as_string();
  marker_scale_x_ = this->get_parameter("teleop_cmd_marker_scale_x").as_double();
  marker_scale_y_ = this->get_parameter("teleop_cmd_marker_scale_y").as_double();
  marker_scale_z_ = this->get_parameter("teleop_cmd_marker_scale_z").as_double();
  
  // Get filter parameters
  filter_frequency_ = this->get_parameter("filter_frequency").as_double();
  filter_mincutoff_ = this->get_parameter("filter_mincutoff").as_double();
  filter_beta_ = this->get_parameter("filter_beta").as_double();
  
  // Update filter initialization with parameters
  filter_linear_x_ = OneEuroFilter(filter_frequency_, filter_mincutoff_, filter_beta_);
  filter_linear_y_ = OneEuroFilter(filter_frequency_, filter_mincutoff_, filter_beta_);
  filter_linear_z_ = OneEuroFilter(filter_frequency_, filter_mincutoff_, filter_beta_);

  // Create subscription to hand_landmarks (PointCloud2)
  // Handles both 2D (z=0) and 3D (z≠0) automatically
  hand_landmarks_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "hand_landmarks",
    10,
    std::bind(&HandJoystickInterface::handLandmarksCallback, this, std::placeholders::_1)
  );

  // Create publishers
  cmd_vel_pub_ = this->create_publisher<extender_msgs::msg::TeleopCommand>("teleop_cmd", 10);
  
  if (enable_marker_visualization_) {
    teleop_cmd_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(cmd_vel_marker_topic, 10);
  }

  RCLCPP_INFO(this->get_logger(), "Hand Joystick Interface initialized");
  RCLCPP_INFO(this->get_logger(), "Dead zone: %.2f", dead_zone_);
  RCLCPP_INFO(this->get_logger(), "XY Saturation zone: %.2f", saturation_zone_);
  RCLCPP_INFO(this->get_logger(), "Depth Saturation zone: %.2f", depth_saturation_zone_);
  RCLCPP_INFO(this->get_logger(), "Tracking landmark index: %d", landmark_index_);
  RCLCPP_INFO(this->get_logger(), "Output rounding decimals: %d", output_round_decimals_);
  RCLCPP_INFO(this->get_logger(), "Marker visualization: %s", enable_marker_visualization_ ? "enabled" : "disabled");
  if (enable_marker_visualization_) {
    RCLCPP_INFO(this->get_logger(), "Teleop_cmd marker topic: %s (frame_id: %s)", cmd_vel_marker_topic.c_str(), marker_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Teleop_cmd marker scale: [%.3f, %.3f, %.3f]", marker_scale_x_, marker_scale_y_, marker_scale_z_);
  }
  RCLCPP_INFO(this->get_logger(), "Note: Z-coordinate will be 0 for 2D tracking, non-zero for 3D");
  RCLCPP_INFO(this->get_logger(), "One Euro Filter enabled: frequency=%.1f Hz, mincutoff=%.2f Hz, beta=%.2f",
              filter_frequency_, filter_mincutoff_, filter_beta_);
}

void HandJoystickInterface::handLandmarksCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Check if we have enough landmarks (MediaPipe hand has 21 landmarks)
  const size_t num_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (num_points == 0 || static_cast<int>(num_points) <= landmark_index_) {
    RCLCPP_WARN(this->get_logger(), "Received insufficient hand landmark data (got %zu points, need index %d)",
                num_points, landmark_index_);
    return;
  }

  // Extract x, y, z coordinates from the specified landmark
  // MediaPipe normalizes x,y coordinates to [0, 1]
  // z is 0 for 2D tracking, or absolute depth in meters for 3D
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (int i = 0; i < landmark_index_; ++i) {
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  double x = static_cast<double>(*iter_x);
  double y = static_cast<double>(*iter_y);
  double z = static_cast<double>(*iter_z);

  // Set reference position on first callback
  if (!reference_set_) {
    reference_position_.x = x;
    reference_position_.y = y;
    reference_position_.z = z;
    reference_set_ = true;
    RCLCPP_INFO(this->get_logger(), "Reference position set: (%.3f, %.3f, %.3f)", x, y, z);
    return;
  }

  // Calculate relative position from reference
  double dx = x - reference_position_.x;
  double dy = y - reference_position_.y;
  double dz = z - reference_position_.z;

  // Check dead zone
  if (isInDeadZone(dx, dy, dz)) {
    // Publish zero velocity in dead zone
    auto teleop_msg = extender_msgs::msg::TeleopCommand();
    teleop_msg.mode = extender_msgs::msg::TeleopCommand::TRANSLATION;
    
    // Apply filtering to maintain smooth transition to zero
    double timestamp = this->now().seconds();
    teleop_msg.twist.linear.x = filter_linear_x_.filter(teleop_msg.twist.linear.x, timestamp);
    teleop_msg.twist.linear.y = filter_linear_y_.filter(teleop_msg.twist.linear.y, timestamp);
    teleop_msg.twist.linear.z = filter_linear_z_.filter(teleop_msg.twist.linear.z, timestamp);
    
    cmd_vel_pub_->publish(teleop_msg);
    if (enable_marker_visualization_) {
      publishMarker(teleop_msg);
    }
    return;
  }

  // Calculate and publish velocity command
  auto teleop_msg = extender_msgs::msg::TeleopCommand();
  teleop_msg.twist = calculateVelocityCommand(dx, dy, dz);
  teleop_msg.mode = extender_msgs::msg::TeleopCommand::TRANSLATION;
  
  // Apply One Euro filter to smooth the velocity
  double timestamp = this->now().seconds();
  teleop_msg.twist.linear.x = filter_linear_x_.filter(teleop_msg.twist.linear.x, timestamp);
  teleop_msg.twist.linear.y = filter_linear_y_.filter(teleop_msg.twist.linear.y, timestamp);
  teleop_msg.twist.linear.z = filter_linear_z_.filter(teleop_msg.twist.linear.z, timestamp);
  
  cmd_vel_pub_->publish(teleop_msg);
  if (enable_marker_visualization_) {
    publishMarker(teleop_msg);
  }

  RCLCPP_DEBUG(this->get_logger(), "Delta: (%.3f, %.3f, %.3f) -> Cmd: (%.2f, %.2f, %.2f)",
               dx, dy, dz, teleop_msg.twist.linear.x, teleop_msg.twist.linear.y, teleop_msg.twist.linear.z);
}

geometry_msgs::msg::Twist HandJoystickInterface::calculateVelocityCommand(
  double x, double y, double z)
{
  auto twist_msg = geometry_msgs::msg::Twist();

  // Normalize each axis independently between -1 and 1
  // Output saturates at ±1 when distance exceeds saturation_zone
  auto normalize_axis = [](double value, double saturation) -> double {
    if (std::abs(value) >= saturation) {
      return (value > 0.0) ? 1.0 : -1.0;
    } else {
      return value / saturation;
    }
  };

  auto round_value = [this](double value) -> double {
    double factor = std::pow(10.0, static_cast<double>(output_round_decimals_));
    return std::round(value * factor) / factor;
  };

  // Normalized output between -1 and 1
  twist_msg.linear.x = round_value(normalize_axis(x, saturation_zone_));
  twist_msg.linear.y = round_value(normalize_axis(y, saturation_zone_));
  twist_msg.linear.z = round_value(normalize_axis(z, depth_saturation_zone_));

  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  return twist_msg;
}

bool HandJoystickInterface::isInDeadZone(double x, double y, double z)
{
  double distance = std::sqrt(x * x + y * y + z * z);
  return distance < dead_zone_;
}

void HandJoystickInterface::publishMarker(const extender_msgs::msg::TeleopCommand& teleop_msg)
{
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.stamp = this->now();
  marker_msg.header.frame_id = marker_frame_id_;
  marker_msg.ns = "teleop_cmd";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::ARROW;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.scale.x = marker_scale_x_;
  marker_msg.scale.y = marker_scale_y_;
  marker_msg.scale.z = marker_scale_z_;
  marker_msg.color.a = 1.0;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;
  
  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  end.x = teleop_msg.twist.linear.x;
  end.y = teleop_msg.twist.linear.y;
  end.z = teleop_msg.twist.linear.z;
  
  marker_msg.points = {start, end};
  teleop_cmd_marker_pub_->publish(marker_msg);
}

} // namespace hand_joystick_interface
