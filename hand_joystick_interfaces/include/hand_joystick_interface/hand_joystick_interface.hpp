#ifndef HAND_JOYSTICK_INTERFACE_HPP
#define HAND_JOYSTICK_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <extender_msgs/msg/teleop_command.hpp>
#include "hand_joystick_interface/one_euro_filter.hpp"

namespace hand_joystick_interface
{

class HandJoystickInterface : public rclcpp::Node
{
public:
  HandJoystickInterface();
  ~HandJoystickInterface() = default;

private:
  // Unified callback (handles both 2D with z=0 and 3D with z≠0)
  void handLandmarksCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // Unified velocity calculation
  geometry_msgs::msg::Twist calculateVelocityCommand(double x, double y, double z);
  
  // Dead zone check (3D distance, works for 2D when z=0)
  bool isInDeadZone(double x, double y, double z);
  
  // Publish visualization marker
  void publishMarker(const extender_msgs::msg::TeleopCommand& teleop_msg);
  
  // ROS 2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr hand_landmarks_sub_;
  rclcpp::Publisher<extender_msgs::msg::TeleopCommand>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr teleop_cmd_marker_pub_;

  // Parameters
  double dead_zone_;
  double saturation_zone_;  // XY saturation distance
  double depth_saturation_zone_;  // Z saturation distance
  int landmark_index_;
  int output_round_decimals_;
  bool enable_marker_visualization_;
  std::string marker_frame_id_;
  double marker_scale_x_;
  double marker_scale_y_;
  double marker_scale_z_;
  
  // State
  geometry_msgs::msg::Point reference_position_;
  bool reference_set_;

  // One Euro Filters for cmd_vel smoothing
  OneEuroFilter filter_linear_x_;
  OneEuroFilter filter_linear_y_;
  OneEuroFilter filter_linear_z_;

  // Filter parameters
  double filter_frequency_;
  double filter_mincutoff_;
  double filter_beta_;
};

} // namespace hand_joystick_interface

#endif // HAND_JOYSTICK_INTERFACE_HPP
