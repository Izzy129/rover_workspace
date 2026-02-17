# urdf_viewer

Generic URDF visualization tool. Launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 to view any URDF/xacro file with interactive joint sliders.

## Build

```bash
colcon build --packages-up-to urdf_viewer
source install/setup.bash
```

## Usage

```bash
# View the default arm (sliding_arm.urdf.xacro from arm_description)
ros2 launch urdf_viewer view_urdf.launch.py

# View any URDF file
ros2 launch urdf_viewer view_urdf.launch.py urdf_file:=src/arm/urdf_viewer/urdf/robot.urdf

# View a xacro file
ros2 launch urdf_viewer view_urdf.launch.py urdf_file:=/absolute/path/to/robot.urdf.xacro

# Without the GUI joint sliders
ros2 launch urdf_viewer view_urdf.launch.py gui:=false
```

## Setting Up RViz Displays

RViz opens with a blank config. To visualize your robot:

1. In the **Displays** panel (left side), click **Add** at the bottom
2. Add a **RobotModel** display:
   - Set **Description Source** to `Topic`
   - Set **Description Topic** to `/robot_description`
3. Add a **TF** display to see the transform frames
4. In **Global Options** (top of Displays panel), set **Fixed Frame** to your URDF's root link (commonly `world` or `base_link`)
5. Optionally add a **Grid** display for spatial reference

Once configured, save your RViz setup via **File > Save Config As** so you can reuse it later.

## Launch Arguments

| Argument    | Default                                        | Description                              |
|-------------|-------------------------------------------------|------------------------------------------|
| `urdf_file` | `arm_description/urdf/sliding_arm.urdf.xacro`  | Absolute path to `.urdf` or `.xacro` file |
| `gui`       | `true`                                          | Launch joint slider GUI (`true`/`false`)  |
