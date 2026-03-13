# Genesis Arm Control Stack

ROS2 Jazzy arm control stack using Genesis (MuJoCo backend) for simulation. Built for the URC rover arm with 5 controllable joints.

**Simulator:** Genesis 0.4.1 with MuJoCo backend (NOT Gazebo)
**No MoveIt2** -- uses a lightweight custom controller with analytical IK planned for later.

## Architecture

```
keyboard_teleop_node  -->  arm_controller_node  -->  genesis_bridge_node
     (input)               (limits/safety)            (sim interface)
```

Three nodes connected by standard ROS2 topics:

| Topic             | Type                          | From                 | To                   |
|-------------------|-------------------------------|----------------------|----------------------|
| `/cmd_joint_vels` | `sensor_msgs/msg/JointState`  | keyboard_teleop_node | arm_controller_node  |
| `/target_joints`  | `sensor_msgs/msg/JointState`  | arm_controller_node  | genesis_bridge_node  |
| `/joint_states`   | `sensor_msgs/msg/JointState`  | genesis_bridge_node  | arm_controller_node  |

The genesis bridge is **dev-only**. On the real rover (Jetson Nano), a hardware driver node replaces it using the same topic interface.

## Arm Joints

| Joint Name     | Type      | Axis | Range                   |
|----------------|-----------|------|-------------------------|
| `base_slide`   | Prismatic | X    | -0.5 to 0.5 m          |
| `shoulder`     | Revolute  | X    | -1.5708 to 1.5708 rad  |
| `elbow`        | Revolute  | X    | 0.0 to 2.6 rad         |
| `wrist_twist`  | Revolute  | Z    | -3.14159 to 3.14159 rad|
| `claw`         | Revolute  | --   | 0.0 to 1.2 rad         |

## Packages

| Package                    | Build Type   | Purpose                                  |
|----------------------------|--------------|------------------------------------------|
| `genesis_arm_description`  | ament_cmake  | MJCF arm model (`mjcf/arm.xml`) + meshes |
| `genesis_arm_teleop`       | ament_python | Keyboard teleoperation node (evdev)      |
| `genesis_arm_control`      | ament_python | Controller with joint limits + smoothing |
| `genesis_arm_bridge`       | ament_python | Genesis/MuJoCo simulation interface      |
| `genesis_arm_bringup`      | ament_cmake  | Launch files for the full stack          |

## Prerequisites

- ROS2 Jazzy (sourced: `source /opt/ros/jazzy/setup.bash`)
- Genesis 0.4.1 + evdev in the `genesis-env` venv (activate: `rover` alias or `source ~/genesis-env/bin/activate`)
- NVIDIA GPU with CUDA (Genesis uses `gs.cuda` backend)
- User must be in the `input` group for keyboard teleop (evdev reads `/dev/input/` directly):

```bash
sudo usermod -aG input $USER
# Log out and back in for the group change to take effect
```

## Build

From the workspace root (`~/rover_workspace`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to genesis_arm_bringup
source install/setup.bash
```

To build only specific packages:

```bash
colcon build --packages-select genesis_arm_bridge genesis_arm_control genesis_arm_teleop
```

## Running

**Note:** The bridge and teleop nodes use venv wrapper scripts (`genesis_bridge_venv`, `keyboard_teleop_venv`) that invoke the `~/genesis-env` Python interpreter. This is required because ROS2 entry points use system Python, which doesn't have Genesis or evdev installed.

### Option 1: Full Teleop Stack (single command)

```bash
source /opt/ros/jazzy/setup.bash
source ~/rover_workspace/install/setup.bash
ros2 launch genesis_arm_bringup teleop.launch.py
```

This launches all three nodes. The keyboard teleop uses evdev for input (no separate terminal needed).

### Option 2: Run Nodes Individually (recommended for debugging)

**Terminal 1 -- Genesis bridge:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/rover_workspace/install/setup.bash
ros2 run genesis_arm_bridge genesis_bridge_venv
```

**Terminal 2 -- Controller:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/rover_workspace/install/setup.bash
ros2 run genesis_arm_control arm_controller --ros-args \
  --params-file ~/rover_workspace/install/genesis_arm_control/share/genesis_arm_control/config/joint_limits.yaml
```

**Terminal 3 -- Keyboard teleop:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/rover_workspace/install/setup.bash
ros2 run genesis_arm_teleop keyboard_teleop_venv
```

### Option 3: Sim Only (no teleop)

```bash
source /opt/ros/jazzy/setup.bash
source ~/rover_workspace/install/setup.bash
ros2 launch genesis_arm_bringup sim.launch.py
```

## Keyboard Controls

The teleop node uses evdev to read raw key press/release events from `/dev/input/`. This gives true simultaneous key support with no delay -- keys are active exactly while physically held.

```
a / d   -->  base slide left / right
w / s   -->  shoulder up / down
q / e   -->  elbow up / down
r / f   -->  wrist twist CW / CCW
o / c   -->  claw open / close
SPACE   -->  emergency stop (all velocities to zero)
Ctrl+C  -->  clean shutdown
```

Multiple keys can be held simultaneously (e.g., `w` + `d` to move shoulder and slide at the same time).

## Debugging

Check topics are active:
```bash
ros2 topic list
ros2 topic hz /joint_states        # expect ~100 Hz
ros2 topic hz /cmd_joint_vels      # expect ~20 Hz
ros2 topic echo /joint_states      # see current positions/velocities
ros2 topic echo /target_joints     # see controller output
```

Switch control mode at runtime:
```bash
# Switch to autonomous mode (True) or teleop mode (False)
ros2 service call /set_control_mode std_srvs/srv/SetBool "{data: false}"
```

## Deployment: Dev Machine vs Jetson Nano

| Machine     | Runs                                             | Purpose            |
|-------------|--------------------------------------------------|--------------------|
| Dev laptop  | All 3 nodes (or just genesis_bridge)             | Simulation/testing |
| Jetson Nano | keyboard_teleop + arm_controller + hardware node | Real rover         |

The topic interface is identical in both cases. A hardware driver node subscribes to `/target_joints` and publishes `/joint_states` from real encoders -- drop-in replacement for the genesis bridge.

Both machines must share the same `ROS_DOMAIN_ID` (default: 42) for cross-machine ROS2 discovery.

## File Structure

```
genesis_test/
├── genesis_arm_description/
│   ├── mjcf/arm.xml              # MuJoCo arm model (5 joints + coupled claw)
│   └── meshes/                   # For future CAD-derived meshes
├── genesis_arm_teleop/
│   ├── scripts/
│   │   └── keyboard_teleop_venv  # Venv wrapper script
│   └── genesis_arm_teleop/
│       └── keyboard_node.py      # evdev-based keyboard input
├── genesis_arm_control/
│   ├── config/joint_limits.yaml  # Joint position/velocity limits
│   └── genesis_arm_control/
│       ├── controller_node.py    # Velocity smoothing + limit enforcement
│       └── kinematics.py         # Stub for future analytical IK
├── genesis_arm_bridge/
│   ├── scripts/
│   │   └── genesis_bridge_venv   # Venv wrapper script
│   └── genesis_arm_bridge/
│       └── bridge_node.py        # Genesis sim loop at 100 Hz
└── genesis_arm_bringup/
    ├── launch/
    │   ├── teleop.launch.py      # Full stack: bridge + controller + teleop
    │   └── sim.launch.py         # Bridge only
    └── config/arm_params.yaml    # Shared params (placeholder)
```

## Joint Limits Configuration

Edit `genesis_arm_control/config/joint_limits.yaml` to change limits. The controller reads these as ROS2 parameters. Key settings:

- `min_position` / `max_position` -- position clamps (meters or radians)
- `max_velocity` -- velocity clamp
- `velocity_smoothing` -- exponential smoothing alpha (0.0 = no smoothing, 1.0 = no filtering). Default: 0.8

After editing, rebuild and re-source:
```bash
colcon build --packages-select genesis_arm_control
source install/setup.bash
```

## Future Work (Phase 6)

- Analytical IK in `kinematics.py` (prismatic base + 2-link planar solver)
- Autonomous planner node subscribing to vision detections
- Controller already has `/cmd_joint_vels_auto` subscriber and `/set_control_mode` service wired up for this
