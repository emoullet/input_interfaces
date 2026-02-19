#include <rclcpp/rclcpp.hpp>
#include "hand_joystick_interface/hand_joystick_interface.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<hand_joystick_interface::HandJoystickInterface>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
