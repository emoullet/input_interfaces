# Face Interface

Control a robot with PlayAbility's virtual Xbox controller and ROS 2 Jazzy. This
package starts PlayAbility, the controller driver and `joystick_mapper`:

```text
PlayAbility → /joy/playability → joystick_mapper
            → /face_joystick_cartesian_command → cartesian_manager → /cartesian_command
```

The face launch does not start `cartesian_manager` or a robot controller.

## Setup

Install ROS 2 Jazzy, then download the Linux AMD64 `.deb` package from the
[PlayAbility releases page](https://github.com/playAbilityTech/playAbility-desktop-releases/releases).
Install the downloaded package, replacing the filename below with the version
you downloaded:

```bash
cd ~/Téléchargements
sudo dpkg -i ./PlayAbility-2.6.0-linux-amd64.deb
```

If `dpkg` reports missing dependencies, run `sudo apt --fix-broken install`.
On Ubuntu 24.04, configure the AppArmor profile below so PlayAbility can use
its Electron sandbox when the installed path contains spaces:

```bash
sudo tee /etc/apparmor.d/playability >/dev/null <<'EOF'
abi <abi/4.0>,
include <tunables/global>

profile playability "/opt/PlayAbility Adaptive Software/playability" flags=(unconfined) {
  userns,
}
EOF

sudo apparmor_parser -r /etc/apparmor.d/playability
```

This profile allows PlayAbility to create user namespaces; it does not apply
additional AppArmor confinement to PlayAbility or change the system-wide
restriction. Associate login links with the Debian package and start the app:

```bash
gio mime x-scheme-handler/playability playability.desktop
playability
```

Enable PlayAbility's virtual Xbox controller before starting the ROS driver.
You can import a reference profile by clicking the
[PlayAbility reference profile link](https://www.playability.gg/profile/92e7462a-fa81-4cb9-b297-55512ce2b5a6).
Select **Auto-start face detection** in PlayAbility so the interface starts
without additional manual steps.

Build the package and its dependencies:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to face_interface \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

In each new terminal, run:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Launch

Start PlayAbility, the driver and the mapper:

```bash
ros2 launch face_interface face_interface.launch.py
```

This uses `config/explorer_face_params.yaml`. It starts the Debian-installed
`playability` executable and the mapper immediately, then starts
`joy/game_controller_node` after three seconds. The delay does not check whether
PlayAbility is ready. If PlayAbility is already running, add
`start_playability:=false`; the driver then starts immediately.

| Argument | Default | Use |
| --- | --- | --- |
| `config_file` | `explorer_face_params.yaml` | Driver and mapper YAML file. |
| `start_playability` | `true` | Set to `false` if PlayAbility is already running. |
| `start_joy` | `true` | Set to `false` to provide Joy messages yourself. |
| `start_mapper` | `true` | Set to `false` if robot bringup starts the mapper. |

The driver uses the mapper's `joy_topic` from the selected YAML. Change
`joy_playability.ros__parameters.device_name` if the virtual controller has a
different name; list devices with `ros2 run joy joy_enumerate_devices`.

## Profiles and controls

Choose a profile by passing its installed path as `config_file`:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_snake_explorer.yaml"
ros2 launch face_interface face_interface.launch.py \
  config_file:="$face_config"
```

Replace the filename with one from this table. Each profile starts in B1.

| File | Robot | Modes | Mode button | Gripper button | Snake button |
| --- | --- | --- | --- | --- | --- |
| `explorer_face_params.yaml` (default) | Explorer | B1, B2, B3 | 2 | 1 | 0, hold |
| `joystick_2d_baseline_explorer.yaml` | Explorer | B1, B2, B3 | 2 | 1 | Disabled |
| `joystick_2d_snake_explorer.yaml` | Explorer | B1, B2 | 2 | 1 | 0, hold |
| `joystick_2d_baseline_kinova.yaml` | Kinova | B1, B2, B3 | 2 | 1 | Disabled |
| `joystick_2d_snake_kinova.yaml` | Kinova | B1, B2 | 2 | 1 | 0, hold |

A press cycles local modes; holding the button does not cycle repeatedly. The
mode maps are the same in all five files:

| Mode | Joy axis 0 | Joy axis 1 | Other components |
| --- | --- | --- | --- |
| B1 | Linear Y | Linear X | Zero |
| B2 | Angular Z | Linear Z | Zero |
| B3 | Angular X | Angular Y | Zero |

All gains are `1.0`. The mapper applies a per-axis dead zone of `0.2`, then
scales linearly to saturation at input magnitude `1.0`. The driver applies no
dead zone and repeats at `50 Hz`. Missing axes and buttons are treated as zero
and released. Linear motion uses the base-frame convention; angular motion in B2
and B3 uses `effector_frame` for Explorer and `end_effector_link` for Kinova.

The gripper button toggles between open and closed positions: `0.2` / `1.1`
for Explorer and `0.0` / `0.8` for Kinova. A Snake button sends
`geometric/snake` while held and `geometric/both` on release. Snake requires a
running `cartesian_manager`; it is not enabled at startup. Baseline profiles
send no Snake request and do not reset a manager already in Snake mode.

| Topic | Message | Purpose |
| --- | --- | --- |
| `/joy/playability` | `sensor_msgs/msg/Joy` | Driver axes and buttons. |
| `/face_joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | Mapper output. |
| `/joystick_mapper/active_mode` | `std_msgs/msg/String` | Current local mode. |
| `/mode_request` | `std_msgs/msg/String` | Snake requests to the manager. |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | Gripper position. |

The old `/teleop_cmd` and `/face_buttons` outputs are not provided; read raw
buttons from `/joy/playability`.

## Connect to a robot

The current Explorer and Kinova manager configurations listen on
`/physical_joystick_cartesian_command` for the physical joystick and
`/face_joystick_cartesian_command` for the independent face joystick input. Robot
bringup already starts a mapper: pass it the selected face YAML,
then start the face launch with the **same file** and `start_mapper:=false`.

Build the manager and robot dependencies if needed:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to face_interface cartesian_manager \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

For Explorer simulation, launch in one prepared terminal:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/explorer_face_params.yaml"
ros2 launch cartesian_manager explorer.launch.py simulation:=true \
  joystick_config_file:="$face_config"
```

In a second prepared terminal, use the same profile:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/explorer_face_params.yaml"
ros2 launch face_interface face_interface.launch.py \
  config_file:="$face_config" start_mapper:=false
```

To use a different Explorer profile, replace the filename in **both** terminals.
The physical `joy_node` started by robot bringup publishes `/joy`; the face
mapper subscribes to `/joy/playability`.

For Kinova with fake hardware, use either Kinova profile in both terminals.
The manager also needs its tool-frame name changed to match these profiles;
otherwise it discards the entire B2/B3 command, including linear Z. In the first
prepared terminal:

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

In the second prepared terminal:

```bash
face_config="$(ros2 pkg prefix face_interface)/share/face_interface/config/joystick_2d_baseline_kinova.yaml"
ros2 launch face_interface face_interface.launch.py \
  config_file:="$face_config" start_mapper:=false
```

## Check the output

Start the mapper without PlayAbility or the driver:

```bash
ros2 launch face_interface face_interface.launch.py \
  start_playability:=false start_joy:=false
```

In another prepared terminal, observe its output:

```bash
ros2 topic echo /face_joystick_cartesian_command
```

In a third prepared terminal, send a sample for the default profile:

```bash
ros2 topic pub --once /joy/playability sensor_msgs/msg/Joy \
  "{axes: [0.0, 1.0], buttons: []}"
```

The initial B1 mode should produce linear X `1.0` and zero on other axes.
With PlayAbility, inspect `/joy/playability`, `/face_joystick_cartesian_command`,
`/joystick_mapper/active_mode`, and, when the manager runs, `/cartesian_command`.
Check directions, mode changes, gripper and Snake controls in simulation before
using the robot. Explorer's configured input timeout is `0.2 s`; the rate limiter
also affects stopping time. The timeout cannot detect a tracking loss if
PlayAbility continues publishing nonzero Joy values.

The synthetic Joy pipeline tests still assert the previous two-mode default and
`0.05` dead zone; they must be updated for the current default profile. The
four alternative profiles and real PlayAbility/robot behavior are not covered.
To run the current suite after building:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select face_interface --event-handlers console_direct+
colcon test-result --test-result-base build/face_interface --verbose
```
