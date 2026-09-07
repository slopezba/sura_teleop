# sura_teleop

`sura_teleop` provides joystick teleoperation for SURA vehicles.

It starts a joystick driver, reads `sensor_msgs/msg/Joy`, switches ROS 2 Control
controllers when requested, and publishes commands to the active controller.

## Role in SURA

This package is used for manual operation. It does not implement controller
logic itself; it sends setpoints or feedforward commands to controllers from
`sura_controllers`.

Typical flow:

```text
gamepad
  -> joy_node
  -> sura_teleop
  -> controller_manager + active controller topic
```

## Launch

Use:

```bash
ros2 launch sura_teleop teleop.launch.py \
  robot_namespace:=<robot_namespace>
```

To override the robot profile environment explicitly:

```bash
ros2 launch sura_teleop teleop.launch.py \
  robot_namespace:=<robot_namespace> \
  environment:=real
```

The launch file starts:

| Node | Package | Purpose |
| --- | --- | --- |
| `joy_node` | `joy` | Publishes joystick state on `/joy`. |
| `sura_teleop` | `sura_teleop` | Converts joystick input into controller switches and commands. |

## Profiles

The launch file uses `environment` from the launch arguments when it is passed
explicitly. If `environment` is empty, it reads `robot.environment` from:

```text
<robot_namespace>_description/config/bringup_description.yaml
```

Then it loads the matching YAML profile:

```text
config/teleop_params_<robot_namespace>_<environment>.yaml
```

For example:

| Environment source | Profile |
| --- | --- |
| `robot_namespace:=cirtesub environment:=sim` or `robot.environment: sim` | `config/teleop_params_cirtesub_sim.yaml` |
| `robot_namespace:=cirtesub environment:=real` or `robot.environment: real` | `config/teleop_params_cirtesub_real.yaml` |
| `robot_namespace:=bluerov environment:=sim` or `robot.environment: sim` | `config/teleop_params_bluerov_sim.yaml` |
| `robot_namespace:=bluerov environment:=real` or `robot.environment: real` | `config/teleop_params_bluerov_real.yaml` |

If the exact YAML file does not exist, the launch fails with an error instead
of selecting a fallback profile. Topic names are read from the selected YAML as
written. The launch only adapts the YAML node key so the parameters apply to
`/<robot_namespace>/sura_teleop`.

## Inputs And Outputs

### Inputs

| Input | Type | Purpose |
| --- | --- | --- |
| `/joy` | `sensor_msgs/msg/Joy` | Joystick buttons and axes. |
| `/<robot_namespace>/controller/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | Reads current controller state before switching. |
| `/<robot_namespace>/controller/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | Activates/deactivates controllers. |

### AUV Outputs

Depending on the active mode, the node publishes one of these command streams:

| Mode | Topic | Type |
| --- | --- | --- |
| Body force | `/<robot_namespace>/controller/body_force/command` | `geometry_msgs/msg/Wrench` |
| Body velocity | `/<robot_namespace>/controller/body_velocity/setpoint` | `geometry_msgs/msg/Twist` |
| Position hold feedforward | `/<robot_namespace>/controller/position_hold/feedforward` | `geometry_msgs/msg/Twist` |
| Stabilize feedforward | `/<robot_namespace>/controller/stabilize/feedforward` | `geometry_msgs/msg/Wrench` |
| Depth hold feedforward | `/<robot_namespace>/controller/depth_hold/feedforward` | `geometry_msgs/msg/Wrench` |

The teleop also calls roll/pitch enable/disable services for `stabilize` and
`depth_hold` when those modes are active.

### Alpha Manipulator Outputs

When arm mode is selected, the node can publish:

| Output | Type | Purpose |
| --- | --- | --- |
| `alpha_*_forward_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | Joint velocity commands for the selected Alpha arm. |
| `alpha_*_gripper_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | Gripper velocity command. |
| `alpha_*_cartesian_velocity_controller/twist` | `geometry_msgs/msg/TwistStamped` | Cartesian velocity command. |

## Controls

The default mappings are designed for an Xbox-like gamepad.

### Mode Selection

| Control | Action |
| --- | --- |
| Left stick button | Select arm teleop mode. |
| Right stick button | Select AUV teleop mode. |

### AUV Controller Selection

These combinations work only in AUV mode:

| Control | Action |
| --- | --- |
| `RB + X` | Toggle `body_velocity`. |
| `RB + B` | Toggle `position_hold`. |
| `RB + Y` | Toggle `stabilize`. |
| `RB + A` | Toggle `depth_hold`. |
| `RB + LB` | Toggle direct `body_force`. |
| D-pad up/down | Enable/disable roll-pitch control for the active stabilize/depth-hold mode. |

### AUV Motion Commands

When not holding `LB`, the sticks command:

| Axis | Command |
| --- | --- |
| surge axis | forward/back motion |
| sway axis | lateral motion |
| heave axis | vertical motion |
| yaw axis | yaw motion |

When holding `LB`, the teleop uses the roll and pitch axes instead of the normal
translation/yaw command.

The exact axis indices and signs come from the selected YAML profile.

### Arm Mode

These combinations work only in arm mode:

| Control | Action |
| --- | --- |
| `RB + X` | Select Cartesian mode. |
| `RB + A` | Select trajectory mode. |
| `RB + B` | Select joint mode. |
| `RB + D-pad right` | Activate selected mode for the left Alpha arm. |
| `RB + D-pad left` | Activate selected mode for the right Alpha arm. |

In joint mode, the configured Alpha axes and triggers publish joint/gripper
velocity commands. In Cartesian mode, they publish a `TwistStamped` command to
the selected Alpha Cartesian velocity controller.

## Configuration

The YAML profiles define:

- controller names;
- command topics;
- controller-manager services;
- button indices;
- axis indices;
- scale/sign for each command axis;
- feedforward gains;
- deadzone;
- Alpha arm controller topics and frame ids.

Common user edits:

| What to change | Where |
| --- | --- |
| Joystick mapping | `buttons` and `axes`. |
| Direction/sign of an axis | `scales`. |
| Command strength | `scales` and controller feedforward gains. |
| Controller names | `*_controller.name`. |
| Controller topics | `*_controller.command_topic`, `setpoint_topic` or `feedforward_topic`. |

Keep controller names and topics consistent with the robot's ROS 2 Control YAML.

## Quick Checks

Check joystick input:

```bash
ros2 topic echo /joy --once
```

Check active controllers:

```bash
ros2 control list_controllers -c /<robot_namespace>/controller/controller_manager
```

Check one command topic, for example body velocity:

```bash
ros2 topic echo /<robot_namespace>/controller/body_velocity/setpoint
```

## Notes

- `environment:=sim` and `environment:=real` can override the robot profile.
  If they are not passed, `robot.environment` selects the YAML file.
- The node only publishes commands for the currently selected mode.
- Controller switching requires the controller manager services to be available.
- If the joystick layout changes, update the matching YAML profile rather than
  changing the C++ node.
