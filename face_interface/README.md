# Face Interface

ROS 2 Jazzy bringup for face control through PlayAbility. This package installs
launch and configuration files and reuses the existing Cartesian mapper:

```text
PlayAbility
  -> joy/game_controller_node
  -> /joy/playability (sensor_msgs/msg/Joy)
  -> joystick_mapper/joystick_mapper_node
  -> /joystick_cartesian_command (geometry_msgs/msg/TwistStamped)
  -> cartesian_manager
  -> /cartesian_command
  -> qontrol_controller
```

## Prerequisites and build

Use Ubuntu 24.04 with ROS 2 Jazzy. Install PlayAbility separately as the Flatpak
application `com.playability.playAbility`, and enable its virtual Xbox controller.
The launch requires `flatpak` on PATH when `start_playability:=true`; colcon and
rosdep do not install PlayAbility. Check the installation with:

```bash
flatpak info com.playability.playAbility
```

Build from the workspace root:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to face_interface \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Prepare **each new terminal** before using the ROS commands below:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Launch

```bash
ros2 launch face_interface face_interface.launch.py
```

The default launch starts PlayAbility, starts the mapper immediately, and starts
`game_controller_node` three seconds after requesting PlayAbility startup. This
delay does not detect application readiness. If PlayAbility is already running:

```bash
ros2 launch face_interface face_interface.launch.py start_playability:=false
```

With PlayAbility startup disabled, the driver starts immediately. Arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `config_file` | Installed `config/explorer_face_params.yaml` | Shared driver and mapper parameter file. |
| `start_playability` | `true` | Launch the Flatpak application. |
| `start_joy` | `true` | Launch the virtual game-controller driver. |
| `start_mapper` | `true` | Launch the mapper; disable if bringup owns it. |

For custom configuration:

```bash
ros2 launch face_interface face_interface.launch.py config_file:=/absolute/path/to/face.yaml
```

The YAML must use `joy_playability.ros__parameters` for the driver and
`joystick_mapper.ros__parameters` for the mapper. The launch reads the mapper's
`joy_topic` to remap the driver's `joy` output, so changing that field updates
both ends of the connection. Select the device through the driver section's
`device_name`; list available devices with `ros2 run joy joy_enumerate_devices`.

## Default controls and public topics

`explorer_face_params.yaml` remains the default when `config_file` is omitted.
The controls in this section describe that default profile.

The initial local mode is `translation`. Each rising edge on button **11**
alternates between `translation` and `rotation`; holding it does not cycle again.

| Mode | Output X/Y/Z | Other components |
| --- | --- | --- |
| `translation` | Linear `[-axes[3], axes[1], axes[0]]` after dead-zone filtering | Angular zero |
| `rotation` | Angular `[-axes[3], axes[1], axes[0]]` after dead-zone filtering | Linear zero |

The mapper applies a scaled per-axis dead zone of `0.05`, ramps linearly to
`+/-1` at input magnitude `1.0`, and saturates beyond it. The driver dead zone
is `0.0` to avoid filtering twice; its autorepeat rate is `50 Hz`.
Both modes publish `base_link` as their frame. Missing axes produce zero.

Gains are normalized to `1.0`. Physical maximum speeds belong to
`qontrol_controller`; the old angular gain `0.2` is intentionally replaced.
Jaco, Snake, Home, and gripper buttons are disabled. Local mode changes do not
publish geometric requests on `/mode_request`.

| Topic | Type | Content |
| --- | --- | --- |
| `/joy/playability` | `sensor_msgs/msg/Joy` | Complete raw axes and buttons from the driver. |
| `/joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | Mapped normalized command for the manager. |
| `/joystick_mapper/active_mode` | `std_msgs/msg/String` | Current local mode, latched by the mapper. |

This migration removes `face_input_node`, `TeleopCommand`, `/teleop_cmd`, and
the duplicate `/face_buttons` output. Read raw buttons from `/joy/playability`.
The old parameter schema, `norm` filtering, configurable saturation threshold,
rounding, and combination profile are not supported. The original external
package and the combination packages are left unchanged.

## Alternative 2D profiles

Four additional YAML files are installed in the package's `config/` directory.
Select one explicitly with `config_file`; this selects both the driver and
mapper parameters. These files do not replace `explorer_face_params.yaml`.

| File | Robot | Local modes, cycled with button 11 | Gripper open / close |
| --- | --- | --- | --- |
| `joystick_2d_baseline_explorer.yaml` | Explorer | B1, B2, B3 | `0.2` / `1.1` |
| `joystick_2d_snake_explorer.yaml` | Explorer | B1, B2 | `0.2` / `1.1` |
| `joystick_2d_baseline_kinova.yaml` | Kinova | B1, B2, B3 | `0.0` / `0.8` |
| `joystick_2d_snake_kinova.yaml` | Kinova | B1, B2 | `0.0` / `0.8` |

All four profiles start in B1, use a mapper dead zone of `0.2`, and use
`/joy/playability` as input. The driver dead zone remains `0.0` and its
autorepeat rate remains `50 Hz`. Their two input axes are mapped as follows,
with positive unit gains after filtering:

| Local mode | Axis 0 | Axis 1 | Angular frame |
| --- | --- | --- | --- |
| B1 | Linear Y | Linear X | `base_link` |
| B2 | Angular Z | Linear Z | Tool frame |
| B3 (baseline only) | Angular X | Angular Y | Tool frame |

The tool frame is `effector_frame` in the Explorer profiles and
`end_effector_link` in the Kinova profiles. Linear commands always follow the
base-frame convention, including linear Z in B2.

Button **0** toggles the gripper. In the Snake profiles, button **1** requests
`geometric/snake` while held and `geometric/both` on release. Snake is not
enabled automatically at startup: it needs the button press and a running
`cartesian_manager` consuming `/mode_request`. Baseline profiles do not send
Snake requests or reset an already selected manager geometric mode.

For example, start the Explorer Snake profile in a prepared terminal:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_snake_explorer.yaml"
ros2 launch face_interface face_interface.launch.py config_file:="$face_config"
```

Replace the filename with any entry from the table to select another profile.
To return to the default, stop the launch and run it without `config_file`.
Use `start_playability:=false` if the application is already running.
When inspecting the mapper alone, `start_playability:=false start_joy:=false`
allows synthetic Joy input, but Snake shaping requires the manager.

## Cartesian manager integration

An existing manager can consume `/joystick_cartesian_command` through its
enabled `joystick` input. Match `topics.joystick_command` if using custom topics.
The face launch does not start a manager or robot controller.

Explorer bringup already starts a mapper. Build its dependencies if needed:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to face_interface cartesian_manager \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

In one prepared terminal, start simulation with the face mapper configuration:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true \
  joystick_config_file:="$(ros2 pkg prefix face_interface)/share/face_interface/config/explorer_face_params.yaml"
```

In another prepared terminal, start the face source without a second mapper:

```bash
ros2 launch face_interface face_interface.launch.py start_mapper:=false
```

The physical `joy_node` started by Explorer publishes `/joy`, while this mapper
consumes `/joy/playability`. Keep exactly one mapper on the manager input topic.

### Alternative profiles with robot bringup

Pass the **same YAML file** to both launches: `joystick_config_file` selects the
mapper configuration in robot bringup, and `config_file` selects the matching
driver configuration in face bringup. Always disable the second mapper with
`start_mapper:=false`.

For Explorer, in one prepared terminal:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_snake_explorer.yaml"
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true \
  joystick_config_file:="$face_config"
```

In another prepared terminal:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_snake_explorer.yaml"
ros2 launch face_interface face_interface.launch.py \
  config_file:="$face_config" start_mapper:=false
```

Use `joystick_2d_baseline_explorer.yaml` in both terminals for the baseline
condition.

For the current Kinova profiles, B2/B3 publish `end_effector_link`. The supplied
Kinova manager configuration uses `frames.ee_frame: effector_frame`, so it needs
an override to accept that input frame. Save the following override and start
Kinova with fake hardware in one prepared terminal:

```bash
cat > /tmp/kinova_face_manager.yaml <<'YAML'
cartesian_manager:
  ros__parameters:
    frames:
      ee_frame: end_effector_link
YAML
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_baseline_kinova.yaml"
ros2 launch cartesian_manager kinova.launch.py use_simulation:=true \
  joystick_config_file:="$face_config" extra_params:=/tmp/kinova_face_manager.yaml
```

In another prepared terminal:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_baseline_kinova.yaml"
ros2 launch face_interface face_interface.launch.py \
  config_file:="$face_config" start_mapper:=false
```

Use `joystick_2d_snake_kinova.yaml` in both terminals for the Snake condition;
the same frame override applies. Without this override, the manager ignores
the entire B2/B3 command, including B2 linear Z.

## Validation

Automated tests use installed launch/configuration files and synthetic Joy input;
they do not require PlayAbility, a joystick, a camera, or a robot. ROS integration
tests require local DDS communication.

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select face_interface --event-handlers console_direct+
colcon test-result --test-result-base build/face_interface --verbose
```

To inject Joy manually, start only the mapper:

```bash
ros2 launch face_interface face_interface.launch.py \
  start_playability:=false start_joy:=false
```

In a second prepared terminal:

```bash
ros2 topic pub --once /joy/playability sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.0, 0.0, 1.0], buttons: []}"
```

This produces linear X `-1.0` in the initial mode. Observe with
`ros2 topic echo /joystick_cartesian_command` in another terminal.

For manual validation with PlayAbility, inspect `/joy/playability` and
`/joystick_cartesian_command` in separate terminals. Check the device selection,
all raw buttons, mapped directions, dead zone, saturation, and button-11 mode
changes. In simulation, also inspect `/cartesian_command` and verify directions
and stopping after interruption of the Joy stream. The manager's joystick
timeout is `0.2 s` in the supplied Explorer configuration; its rate limiter
affects stopping time. If PlayAbility continues publishing a nonzero value after
losing face tracking, the input timeout cannot detect that tracking loss.

For the alternative profiles, also check the B1/B2/B3 mappings above, gripper
toggle on button 0, and Snake activation/release on button 1. The automated
pipeline tests currently validate the default profile; they do not validate
the four alternative profiles or physical gripper motion.

After changes, rebuild, source `install/setup.bash` again, and restart the running
face driver, mapper, and manager as applicable. No web page refresh is required.
