# Navigation

nav team put ur packages here

do not copy install, log, or build folders here!
only source files (like package.xml)


once copied, go back to root directory, and resolve dependencies with rosdep

```
rosdep install --from-paths src --ignore-src -r -y
```

then build with
```
colcon build
```

---

## Rover Body Gazebo Simulation

Packages: `rover_description` (URDF + meshes) and `rover_gazebo` (Gazebo + ros2_control).

The URDF source of truth lives in `src/urdf_models/rigid_body/` and is installed into
`rover_description` at build time via its CMakeLists.txt.

### Build

```bash
# From workspace root
colcon build --packages-up-to rover_gazebo
source install/setup.bash
```

### Launch Gazebo

```bash
ros2 launch rover_gazebo rover_gazebo.launch.py
```

The rover spawns above the ground plane and drops onto it. Wait for both controllers
to activate (you'll see them appear in the Gazebo terminal output) before driving.

### Drive with keyboard teleop

In a **separate terminal** (must have keyboard focus to receive input):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel
```

> **Important:** `-p stamped:=true` is required. Jazzy's `diff_drive_controller`
> only accepts `TwistStamped` — plain `Twist` messages are silently ignored.

Default teleop keys:
- `i` — forward
- `,` — backward
- `j` / `l` — turn left / right
- `k` — stop
- `q` / `z` — increase / decrease speed

### Verify controllers are active

```bash
ros2 control list_controllers
# Expected output:
# diff_drive_controller   diff_drive_controller/DiffDriveController      active
# joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
```

### Tuning wheel parameters

Edit [`rover_gazebo/config/rover_controllers.yaml`](rover_gazebo/config/rover_controllers.yaml)
to update `wheel_separation` and `wheel_radius` once measured from the CAD model.
Rebuild `rover_gazebo` after any config changes:

```bash
colcon build --packages-select rover_gazebo && source install/setup.bash
```