# arm_teleop

Keyboard teleop control and simulated driver for the rover's 3DOF arm. Built with `ament_cmake_python` so C++ nodes can be added later.

## Nodes

### keyboard_teleop.py

Reads keyboard input and publishes joint commands to `/arm_joint_commands`.

**Must be run with `ros2 run`** (not `ros2 launch`) because it needs direct TTY access for raw keyboard input.

```bash
ros2 run arm_teleop keyboard_teleop.py
```

#### Controls

```
w/s  - slider up/down
a/d  - shoulder left/right
j/l  - elbow left/right
v    - toggle position/velocity mode
0    - zero all joints
q    - quit
```

#### Control Modes

- **Velocity mode** (default): Hold a key to move the joint continuously. Release to decelerate to a smooth stop. The teleop publishes desired velocities in the `velocity` field of JointState.
- **Position mode**: Each key press increments the target position by a fixed step size. The teleop publishes target positions in the `position` field of JointState.

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Desired positions or velocities |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `joint_names` | `[slider, shoulder_1, elbow_1]` | Joint names to command |
| `slider_step` | `0.005` | Position step size for slider (m) |
| `shoulder_step` | `0.03` | Position step size for shoulder (rad) |
| `elbow_step` | `0.03` | Position step size for elbow (rad) |
| `slider_vel` | `0.04` | Velocity target for slider in velocity mode (m/s) |
| `shoulder_vel` | `0.53` | Velocity target for shoulder in velocity mode (rad/s) |
| `elbow_vel` | `0.53` | Velocity target for elbow in velocity mode (rad/s) |
| `slider_limits` | `[-0.25, 0.0]` | Slider position limits (m) |
| `shoulder_limits` | `[-3.14159, 3.14159]` | Shoulder position limits (rad) |
| `elbow_limits` | `[-3.14159, 3.14159]` | Elbow position limits (rad) |
| `publish_rate` | `20.0` | Command publish rate (Hz) |

### sim_driver.py

Simulated arm driver with trapezoidal velocity profiles. Receives commands on `/arm_joint_commands` and publishes simulated joint states to `/joint_states`.

This node is a **drop-in placeholder** for a real hardware driver. It simulates motor behavior by smoothly accelerating, cruising, and decelerating joints rather than teleporting them instantly.

Supports two control modes (auto-detected from incoming messages):
- **Position commands** (`msg.position` populated): Moves to target with trapezoidal velocity profile
- **Velocity commands** (`msg.velocity` populated): Tracks desired velocity with acceleration limits, decelerates to stop when velocity command goes to zero

```bash
# Usually launched via arm_bringup, but can be run standalone:
ros2 run arm_teleop sim_driver.py
```

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Position or velocity commands |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Simulated joint positions at 20Hz |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slider_max_vel` | `0.05` | Max slider velocity (m/s) |
| `slider_max_accel` | `0.1` | Max slider acceleration (m/s^2) |
| `shoulder_max_vel` | `0.67` | Max shoulder velocity (rad/s) |
| `shoulder_max_accel` | `2.0` | Max shoulder acceleration (rad/s^2) |
| `elbow_max_vel` | `0.67` | Max elbow velocity (rad/s) |
| `elbow_max_accel` | `2.0` | Max elbow acceleration (rad/s^2) |
| `slider_limits` | `[-0.25, 0.0]` | Slider position limits (m) |
| `shoulder_limits` | `[-3.14159, 3.14159]` | Shoulder limits (rad) |
| `elbow_limits` | `[-3.14159, 3.14159]` | Elbow limits (rad) |

### gazebo_bridge.py

Translates `/arm_joint_commands` into the `Float64MultiArray` format expected by the `ros2_control` `ForwardCommandController` running inside Gazebo.

This node is launched automatically by `arm_gazebo` with a 5-second delay to ensure `arm_vel_controller` is active before commands flow through. It is not used in the lightweight `arm_bringup` sim.

```bash
# Usually launched via arm_gazebo, but can be run standalone:
ros2 run arm_teleop gazebo_bridge.py
```

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Velocity commands from keyboard teleop |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_vel_controller/commands` | `std_msgs/Float64MultiArray` | Velocity commands for ros2_control |

#### Parameters

Shares `config/teleop_params.yaml` with the other teleop nodes. Joint name order must match the `arm_vel_controller` configuration in `arm_gazebo`.

## Configuration

All parameters are in [`config/teleop_params.yaml`](config/teleop_params.yaml), loaded by the launch files.

## Build

```bash
colcon build --packages-select arm_teleop
source install/setup.bash
```
