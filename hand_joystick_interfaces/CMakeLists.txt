cmake_minimum_required(VERSION 3.8)
project(hand_joystick_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(extender_msgs REQUIRED)

# Add the executable for the hand joystick interface node
add_executable(hand_joystick_node
  src/main.cpp
  src/hand_joystick_interface.cpp
)

# Package dependencies
ament_target_dependencies(hand_joystick_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  extender_msgs
)

target_include_directories(hand_joystick_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hand_joystick_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS 
  hand_joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
