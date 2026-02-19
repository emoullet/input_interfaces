# Hand Joystick 3D Interface

## Overview

This node converts hand landmarks with depth information into 3D velocity commands (Twist messages), enabling intuitive 3D control using hand gestures. It builds on the depth inference capabilities to provide full 3-axis translational control.

## Features

- **3D Hand Tracking**: Uses 2D hand landmarks combined with depth inference for full 3D position tracking
- **Configurable Dead Zone**: Prevents noise and drift when hand is stationary
- **Independent Axis Saturation**: XY and depth use different saturation zones for optimal control
- **Landmark Selection**: Track any of the 21 MediaPipe hand landmarks (default: index finger tip)
- **Automatic Reference**: Sets reference position on first detection

## Architecture

```
┌─────────────────────┐
│  Camera Image       │
│  (RGB)              │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────────────────┐
│ hand_landmarks_depth_inference  │
│ (MediaPipe + Depth Estimation)  │
└──────────┬──────────────────────┘
           │
           │ /hand_landmarks_depth (PointCloud)
           │   - x, y: normalized 2D coords
           │   - z: absolute depth (meters)
           │
           ▼
     ┌──────────────────────────┐
     │ hand_joystick_3d_node    │
     │ (3D Velocity Controller) │
     └──────────┬───────────────┘
                │
                ▼ /cmd_vel (Twist)
         ┌─────────────┐
         │  Robot or   │
         │  Application│
         └─────────────┘
```

## Dependencies

- **mediapipe_mocap**: Provides depth inference node
- **ROS 2 Messages**: geometry_msgs, sensor_msgs, std_msgs

## Usage

### Test with Video and Fractional Teleoperation Controller

Run the complete hand gesture-based teleoperation system with the fractional teleoperation controller. The launch file is located in the `fractional_teleoperation` package and automatically generates `robot_description` from URDF:

```bash
# Build the packages
cd ~/gits/new_extender/extender_workspace
colcon build --packages-up-to fractional_teleoperation hand_joystick_interfaces

# Source the workspace
source install/setup.bash

# Launch the complete system
# For full functionality with real robot support, build explorer_description first:
colcon build --packages-up-to explorer_description

# Then launch:
ros2 launch fractional_teleoperation test_video_fractional_teleoperation_launch.py \
  folder_path:=$HOME/test_videos_hands/ \
  fps:=30 \
  gui:=true
```

**Components Launched:**
1. **offline_media_publisher** - Publishes video frames from folder
2. **hand_landmarks_node_depth_inference** - Detects hand landmarks with depth estimation
3. **hand_joystick_3d_interface** - Converts 3D hand position to velocity commands
4. **robot_state_publisher** - Publishes robot state and transforms (auto-generated from URDF)
5. **controller_manager** - ROS 2 control manager
6. **fractional_teleoperation_controller** - Teleoperation controller plugin
7. **rviz2** - 3D visualization (optional, enabled with `gui:=true`)
8. **hand_landmarks_viewer** - Visualizes hand landmarks

**Available Arguments:**
- `folder_path` - Path to video folder (required)
- `fps` - Publishing rate (default: 30)
- `gui` - Enable RViz2 visualization (default: true)
- `simulation` - Use simulation mode (default: false)
- `use_sim_time` - Use simulated clock (default: false)
- `robot_type` - Robot type: explorer or franka (default: explorer)
- `hand_joystick_config` - Hand joystick configuration file
- `controller_config` - Controller configuration file

**Robot Description:**

The launch file automatically generates `robot_description` from URDF/xacro:
- **If `explorer_description` is available** - Loads from `explorer.urdf.xacro` (full functionality)
- **If `explorer_description` is NOT available** - Uses a minimal fallback URDF (hand joystick interface still works)

For complete robot control functionality including the fractional teleoperation controller, build the explorer_stack:
```bash
colcon build --packages-select explorer_description
```

### Test with Video Only

Test the 3D interface without the controller:

```bash
ros2 launch hand_joystick_interfaces test_video_3d_inf_hand_joystick_launch.py \
  folder_path:=$HOME/test_videos_hands/ \
  fps:=30
```

### Monitor Output

View the 3D joystick output:

```bash
ros2 topic echo /cmd_vel
```

Expected output structure:
```
linear:
  x: <-1 to 1>  # Left-right
  y: <-1 to 1>  # Up-down  
  z: <-1 to 1>  # Forward-backward (depth)
```

## Usage

### Quick Start

1. **Build the package**:
   ```bash
   cd extender_workspace
   colcon build --packages-select hand_joystick_interfaces mediapipe_mocap
   source install/setup.bash
   ```

2. **Launch the 3D interface**:
   ```bash
   ros2 launch hand_joystick_interfaces hand_joystick_3d_launch.py
   ```

3. **Register reference position** (in the depth inference node):
   ```bash
   ros2 service call /hand_landmarks_depth_inference_node/register_reference std_srvs/srv/Trigger
   ```

4. **Monitor velocity commands**:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### Test with Offline Video

For testing without a live camera:

```bash
ros2 launch hand_joystick_interfaces test_video_3d_inf_hand_joystick_launch.py \
  folder_path:=/path/to/video/folder
```

This launches:
- Offline video publisher
- Depth inference node
- 3D joystick interface
- Viewer node (for visualization)

## Topics

### Subscribed

- `/hand_landmarks_depth` (sensor_msgs/PointCloud)
  - 21 hand landmarks with:
    - `x`, `y`: normalized 2D coordinates [0, 1]
    - `z`: inferred absolute depth (meters)
  - Published by `hand_landmarks_node_depth_inference`

### Published

- `/cmd_vel` (geometry_msgs/Twist)
  - Linear velocity commands (normalized [-1, 1])
  - `linear.x`: Hand movement in X (left/right)
  - `linear.y`: Hand movement in Y (up/down)
  - `linear.z`: Hand movement in Z (forward/backward based on depth)
  - Angular velocities set to zero

## Configuration

Edit [3d_parameters.yaml](../config/3d_parameters.yaml):

```yaml
hand_joystick_3d_interface:
  ros__parameters:
    dead_zone: 0.05              # Distance threshold for zero output
    saturation_zone: 0.2          # XY distance where output reaches ±1
    depth_saturation_zone: 0.3    # Depth distance (meters) where Z output reaches ±1
    landmark_index: 8             # Which landmark to track (8 = index finger tip)
```

### Parameters Explained

- **dead_zone** (double, default: 0.05)
  - 3D euclidean distance from reference below which output is zero
  - Prevents jitter when holding position
  - Units: normalized for XY, meters for Z

- **saturation_zone** (double, default: 0.2)
  - Distance from reference where XY output saturates at ±1
  - Smaller values = more sensitive control
  - Units: normalized [0, 1]

- **depth_saturation_zone** (double, default: 0.3)
  - Depth change (meters) where Z output saturates at ±1
  - Example: 0.3 means ±30cm from reference = ±1 output
  - Adjust based on workspace size

- **landmark_index** (int, default: 8)
  - Which hand landmark to track (0-20)
  - Common choices:
    - 0: Wrist (stable, less precise)
    - 8: Index finger tip (precise, can be jittery)
    - 9: Index finger MCP (good balance)
    - 12: Middle finger tip

## Control Mapping

The node applies independent normalization to each axis:

```
For XY (2D position):
  output = (current - reference) / saturation_zone
  clamped to [-1, 1]

For Z (depth):
  output = (current_depth - reference_depth) / depth_saturation_zone
  clamped to [-1, 1]
```

### Example Movement Mapping

If using default parameters and tracking index finger tip:

| Hand Movement | Output cmd_vel |
|--------------|----------------|
| Move right 20cm | linear.x = +1.0 |
| Move left 10cm | linear.x = -0.5 |
| Move up 15cm | linear.y = +0.75 |
| Move down 5cm | linear.y = -0.25 |
| Move forward 30cm | linear.z = +1.0 |
| Move backward 15cm | linear.z = -0.5 |
| Small movements < 5cm | All zeros (dead zone) |

## Workflow

1. **Initialization**:
   - Launch depth inference node (must register reference first!)
   - Launch 3D joystick interface
   - Node waits for both landmark and depth data

2. **Reference Setting**:
   - On first valid detection, node captures reference position
   - Both XY coordinates and depth are stored
   - Logged: "Reference position set: (x, y, depth)"

3. **Continuous Operation**:
   - Compute delta from reference for all 3 axes
   - Check if in dead zone (3D distance)
   - Normalize each axis independently
   - Publish Twist message

## Use Cases

### 1. Drone Control
```yaml
saturation_zone: 0.15          # Tighter control
depth_saturation_zone: 0.4     # Larger Z range
landmark_index: 9              # Stable MCP joint
```

### 2. Robot Arm Control
```yaml
saturation_zone: 0.25          # More forgiving
depth_saturation_zone: 0.2     # Precise depth
landmark_index: 8              # Precise fingertip
```

### 3. 3D Navigation/Gaming
```yaml
saturation_zone: 0.2
depth_saturation_zone: 0.25
landmark_index: 0              # Stable wrist tracking
```

## Troubleshooting

### No output despite hand detection

- **Check depth node**: Ensure `hand_landmarks_node_depth_inference` is running
- **Register reference**: Call the register_reference service on the depth node
- **Check topics**: Verify data on `/hand_landmarks_depth` (z values should be non-zero)
- **Check parameters**: Hand might be in dead zone

### Jittery control

- **Increase dead_zone**: e.g., from 0.05 to 0.08
- **Change landmark**: Use more stable landmarks (9 instead of 8)
- **Smooth output**: Add a low-pass filter in your control pipeline

### Z-axis not responding

- **Depth saturation**: Depth changes might be too small
- **Decrease depth_saturation_zone**: Make it more sensitive
- **Check depth data**: `ros2 topic echo /hand_landmarks_depth` (look at z values)

### Control too sensitive/not sensitive enough

- **Adjust saturation zones**: 
  - Smaller = more sensitive (smaller movements give full output)
  - Larger = less sensitive (need larger movements for full output)

## Technical Details

### Dead Zone Calculation

The dead zone uses 3D Euclidean distance:

```cpp
bool isInDeadZone(double dx, double dy, double dz) {
  double distance = sqrt(dx*dx + dy*dy + dz*dz);
  return distance < dead_zone_;
}
```

This creates a spherical dead zone around the reference position.

### Normalization

Each axis is normalized independently:

```cpp
auto normalize_axis = [](double value, double saturation) {
  if (abs(value) >= saturation) {
    return (value > 0) ? 1.0 : -1.0;
  }
  return value / saturation;
};

twist.linear.x = normalize_axis(dx, saturation_zone_);
twist.linear.y = normalize_axis(dy, saturation_zone_);
twist.linear.z = normalize_axis(dz, depth_saturation_zone_);
```

This allows independent control of XY sensitivity vs depth sensitivity.

## Limitations

- **Requires depth inference**: Must run alongside `hand_landmarks_node_depth_inference`
- **Single hand**: Currently tracks only the first detected hand
- **Reference drift**: Re-register reference if tracking quality degrades
- **Depth accuracy**: Limited by monocular depth estimation accuracy (~5-10% error)
- **No rotation**: Only translational control (could be extended with hand orientation)

## Future Enhancements

- [ ] Multi-hand support (one for position, one for orientation)
- [ ] Gesture-based mode switching
- [ ] Dynamic dead zone adjustment based on velocity
- [ ] Rotation control using hand orientation
- [ ] Button/pinch gestures for discrete commands
- [ ] Exponential response curves for fine control

## See Also

- [Hand Landmarks Depth Inference README](../../tools/mediapipe_mocap/mediapipe_mocap/DEPTH_INFERENCE_README.md)
- [2D Hand Joystick Interface](hand_joystick_2d_interface.cpp) - Simpler 2D-only version
