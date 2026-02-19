# arm_bringup

Launch orchestration for the arm teleop system. Brings up `robot_state_publisher`, the sim driver, and RViz2 in a single launch command.

## Usage

```bash
# Build
colcon build --packages-up-to arm_bringup
source install/setup.bash

# Launch full system (sim driver + RViz)
ros2 launch arm_bringup arm_bringup.launch.py

# Without RViz
ros2 launch arm_bringup arm_bringup.launch.py use_rviz:=false

# Without sim driver (for real hardware)
ros2 launch arm_bringup arm_bringup.launch.py use_sim:=false
```

Then in a separate terminal, run the keyboard teleop:

```bash
ros2 run arm_teleop keyboard_teleop.py
```

> **Note:** The keyboard teleop must be run with `ros2 run` (not included in the launch file) because it needs direct TTY access for raw keyboard input.

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim` | `true` | Launch `sim_driver.py` (set `false` when using real hardware) |
| `use_rviz` | `true` | Launch RViz2 with arm visualization config |

## What Gets Launched

| Node | Package | Description |
|------|---------|-------------|
| `robot_state_publisher` | `robot_state_publisher` | Publishes TF from URDF (handles mimic joints) |
| `arm_sim_driver` | `arm_teleop` | Simulated motor driver (when `use_sim:=true`) |
| `rviz2` | `rviz2` | 3D visualization (when `use_rviz:=true`) |

## URDF

Uses the real CAD-derived URDF from the `urdf_viewer` package (`urdf/robot.urdf`). No separate URDF is maintained in this package.

## Debugging

```bash
# Verify topics are active
ros2 topic list
ros2 topic hz /joint_states          # Should be ~20Hz
ros2 topic echo /arm_joint_commands  # Commands from teleop
ros2 topic echo /joint_states        # States from sim driver

# Check TF tree
ros2 run tf2_tools view_frames
```
