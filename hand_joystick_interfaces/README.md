# 2D Hand Joystick Interface

This package provides a ROS 2 interface for controlling robots using 2D hand tracking data from MediaPipe, converting hand landmark positions to joystick-like commands.

## Overview

The `2D_hand_joystick_interface` package processes hand landmark data from the `hand_landmarks_node.py` (part of mediapipe_mocap) and converts it into control commands suitable for robot teleoperation.

## Features

- Subscribes to MediaPipe hand landmarks (PointCloud with 21 landmarks)
- Configurable landmark selection (wrist, finger tips, palm center, etc.)
- Converts 2D hand positions to velocity commands
- Configurable control zones and dead zones
- Reference position tracking for relative control
- Compatible with standard ROS 2 geometry messages

## Nodes

### hand_joystick_node

Subscribes to hand landmark data from `hand_landmarks_node.py` and publishes control commands.

**Subscribed Topics:**
- `/hand_landmarks` (sensor_msgs/msg/PointCloud) - Hand tracking data with 21 landmarks

**Published Topics:**
- `/cmd_vel` (geometry_msgs/msg/Twist) - Velocity commands

**Parameters:**
- `dead_zone`: Dead zone radius to ignore small movements (default: 0.05)
- `saturation_zone`: Distance from reference point where output saturates at ±1 (default: 0.2)
- `landmark_index`: Which landmark to track - 0=wrist, 8=index finger tip, 9=middle finger MCP (default: 8)

**Output Behavior:**
- Output is normalized between -1 and +1 for each axis
- Within `dead_zone`: output = 0
- Between `dead_zone` and `saturation_zone`: output scales linearly from 0 to ±1
- Beyond `saturation_zone`: output saturates at ±1

## MediaPipe Hand Landmarks

The hand_landmarks_node.py publishes 21 landmarks per hand:
- 0: Wrist
- 1-4: Thumb (CMC, MCP, IP, TIP)
- 5-8: Index finger (MCP, PIP, DIP, TIP)
- 9-12: Middle finger (MCP, PIP, DIP, TIP)
- 13-16: Ring finger (MCP, PIP, DIP, TIP)
- 17-20: Pinky (MCP, PIP, DIP, TIP)

Coordinates are normalized to [0, 1] range.

## Usage

### Complete System Test with Video

Test the complete pipeline with video files using the integrated launch file:

```bash
ros2 launch hand_joystick_interfaces test_video_2d_hand_joystick_launch.py folder_path:=$HOME/test_videos_hands/ fps:=30
```

This launch file starts:
1. **video_publisher** - Publishes video frames from a folder
2. **hand_landmarks_node** - Detects hand landmarks using MediaPipe
3. **hand_joystick_node** - Converts hand position to joystick commands
4. **viewer_node** - Visualizes hand landmarks (optional)

### Complete System Launch

Alternatively, start components separately.

First, start the MediaPipe hand landmarks node:

```bash
ros2 launch mediapipe_mocap test_offline_video_hand_landmarks_launch.py folder_path:=$HOME/test_videos_hands/ fps:=30
```

Then start the hand joystick interface:

```bash
ros2 launch 2D_hand_joystick_interface hand_joystick_launch.py
```

### Building

```bash
cd ~/extender_workspace
colcon build --packages-select 2D_hand_joystick_interface
source install/setup.bash
```

### Running

```bash
ros2 run 2D_hand_joystick_interface hand_joystick_node
```

Or with a launch file:

```bash
ros2 launch 2D_hand_joystick_interface hand_joystick_launch.py
```

### Monitoring Output

View the normalized joystick output:

```bash
ros2 topic echo /cmd_vel
```

The output values will be between -1 and +1 for linear.x and linear.y.

## Configuration

Edit the configuration files in the `config/` directory to customize the behavior for different robots or control schemes.

## Dependencies

- rclcpp
- geometry_msgs
- std_msgs
- sensor_msgs
- extender_msgs
