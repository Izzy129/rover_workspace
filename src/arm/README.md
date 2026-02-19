# Arm Subsystem

Teleop control for the rover's 3DOF arm (1 prismatic + 2 revolute joints with parallelogram linkage).

## Setup

> **Note:** Do not copy `install/`, `log/`, or `build/` folders here — only source files (like `package.xml`).

Once packages are added, resolve dependencies and build:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to arm_bringup
source install/setup.bash
```

## Quick Start

### Lightweight sim (no physics, fast startup)

```bash
# Terminal 1: launch robot_state_publisher, sim driver, and RViz
ros2 launch arm_bringup arm_bringup.launch.py

# Terminal 2: keyboard teleop (must be ros2 run, not launch, for TTY access)
ros2 run arm_teleop keyboard_teleop.py
```

### Gazebo physics sim (ros2_control, full physics)

```bash
# Terminal 1: launch Gazebo + controllers + RViz
ros2 launch arm_gazebo arm_gazebo.launch.py

# Terminal 2: keyboard teleop
ros2 run arm_teleop keyboard_teleop.py
```

## Controls

```
w/s  - slider up/down
a/d  - shoulder left/right
j/l  - elbow left/right
v    - toggle position/velocity mode
0    - zero all joints
q    - quit
```

**Velocity mode** (default): Hold a key to move continuously, release to decelerate to a stop. More intuitive for real-time control.

**Position mode**: Each key press increments the target position by a fixed step. The arm moves to the target with a trapezoidal velocity profile.

## Architecture

### Lightweight sim (`arm_bringup`)

```
keyboard_teleop ──/arm_joint_commands──> sim_driver ──/joint_states──> robot_state_publisher ──> RViz
```

### Gazebo physics sim (`arm_gazebo`)

```
keyboard_teleop
    ↓ /arm_joint_commands
gazebo_bridge.py
    ↓ /arm_vel_controller/commands
arm_vel_controller (ForwardCommandController)
    ↓
Gazebo physics
    ↓
joint_state_broadcaster
    ↓ /joint_states
robot_state_publisher
    ↓ /tf
RViz2
```

- **arm_teleop**: Keyboard teleop node, sim driver, and Gazebo bridge
- **arm_bringup**: Launch orchestration for lightweight sim (robot_state_publisher + sim_driver + RViz)
- **arm_gazebo**: Physics-based Gazebo sim with ros2_control
- **urdf_viewer**: Real CAD-derived URDF from Onshape (used as-is)

## Packages

| Package | Description | Details |
|---------|-------------|---------|
| [arm_teleop](arm_teleop/) | Teleop node + sim driver | Keyboard control, simulated motor behavior, Gazebo bridge |
| [arm_bringup](arm_bringup/) | Lightweight sim launch | robot_state_publisher + sim_driver + RViz |
| [arm_gazebo](arm_gazebo/) | Gazebo physics sim | ros2_control, ForwardCommandController, full physics |
| [urdf_viewer](urdf_viewer/) | URDF visualization | Interactive joint sliders, CAD-derived URDF |
| [arm_description](arm_description/) | Gazebo sim (legacy) | Superseded by arm_gazebo |

## Path to Real Hardware

The sim driver (`sim_driver.py`) is a drop-in placeholder. To use real motors:

1. Create a hardware driver node that subscribes to `/arm_joint_commands`, sends commands to motors, reads encoders, and publishes to `/joint_states`
2. Launch with `ros2 launch arm_bringup arm_bringup.launch.py use_sim:=false`
3. Run your hardware driver separately

No changes needed to `keyboard_teleop` or the bringup launch structure.

## Controlled Joints

| Joint | Type | Limits | Description |
|-------|------|--------|-------------|
| `slider` | prismatic | -0.25m to 0m | Linear rail |
| `shoulder_1` | revolute | -pi to +pi | Primary shoulder |
| `elbow_1` | revolute | -pi to +pi | Primary elbow |

4 additional mimic joints (parallelogram linkage) are handled automatically by `robot_state_publisher`.
