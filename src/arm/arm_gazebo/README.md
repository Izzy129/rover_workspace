# arm_gazebo

Physics-based Gazebo (Ignition) simulation for the rover arm using `ros2_control`. This is the high-fidelity sim alternative to the lightweight `sim_driver.py` in `arm_teleop`.

## Quick Start

```bash
# Build
colcon build --packages-up-to arm_gazebo
source install/setup.bash

# Terminal 1: launch Gazebo + controllers + RViz
ros2 launch arm_gazebo arm_gazebo.launch.py

# Terminal 2: keyboard teleop (must be ros2 run, not launch, for TTY access)
ros2 run arm_teleop keyboard_teleop.py
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Launch RViz2 alongside Gazebo |

## Data Flow

```
keyboard_teleop
    ↓ /arm_joint_commands (sensor_msgs/JointState)
gazebo_bridge.py
    ↓ /arm_vel_controller/commands (std_msgs/Float64MultiArray)
arm_vel_controller (ForwardCommandController)
    ↓ joint velocity commands
Gazebo physics
    ↓ simulated joint states
joint_state_broadcaster
    ↓ /joint_states (sensor_msgs/JointState)
robot_state_publisher
    ↓ /tf, /tf_static
RViz2
```

## What Gets Launched

| Node/Process | Description |
|---|---|
| `robot_state_publisher` | Publishes TF from URDF (uses sim time from Gazebo) |
| `gz_sim` | Physics simulation with `gz_ros2_control` plugin |
| `ros_gz_sim create` | Spawns the robot from `/robot_description` topic |
| `parameter_bridge` | Bridges `/clock` from Gazebo → ROS2 (required for `use_sim_time`) |
| `joint_state_broadcaster` | Reads joint states from the simulation and publishes `/joint_states` |
| `arm_vel_controller` | `ForwardCommandController` — forwards velocity commands to simulated joints |
| `gazebo_bridge.py` | Translates `/arm_joint_commands` → `/arm_vel_controller/commands` |
| `rviz2` | 3D visualization (when `use_rviz:=true`) |

## Controller Startup Sequence

Controllers are spawned in a strict order to avoid race conditions:

1. `joint_state_broadcaster` spawns immediately after Gazebo is ready
2. `arm_vel_controller` spawns after `joint_state_broadcaster` exits (via `OnProcessExit`)
3. `gazebo_bridge.py` starts 5 seconds later (via `TimerAction`) to ensure the velocity controller is fully active before commands flow through

## ros2_control Configuration

Defined in `config/arm_controllers.yaml`. Key settings:

- Controller update rate: **100 Hz**
- `joint_state_broadcaster`: publishes all joint states
- `arm_vel_controller`: `ForwardCommandController` commanding the `velocity` interface on joints `[slider, shoulder_1, elbow_1]`

## URDF

Uses `urdf/arm_gazebo.urdf.xacro`, which extends the CAD-derived URDF from `urdf_viewer` with:

- `gz_ros2_control` plugin for hardware interface
- Gazebo-compatible inertia and collision properties

## Mesh Resolution

Gazebo resolves `model://` URIs by searching `GZ_SIM_RESOURCE_PATH`. The launch file sets this variable to the parent of the `urdf_viewer` share directory so that paths like `model://urdf_viewer/urdf/assets/*.stl` resolve correctly to the installed STL files.

## Comparison: Gazebo sim vs sim_driver

| Feature | `arm_gazebo` (this package) | `arm_bringup` + `sim_driver` |
|---|---|---|
| Physics engine | Gazebo Ignition | None (pure kinematics) |
| ros2_control | Yes (`ForwardCommandController`) | No |
| Sim time | Yes (`use_sim_time:=true`) | No |
| Mesh rendering | Gazebo + RViz | RViz only |
| Complexity | Higher | Lower |
| Use case | Full physics validation | Lightweight teleop testing |
