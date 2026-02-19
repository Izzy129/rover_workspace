# urdf_viewer

URDF visualization tool for the rover arm. Launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 with a preconfigured view to visualize the arm URDF with interactive joint sliders.

## Build

```bash
colcon build --packages-up-to urdf_viewer
source install/setup.bash
```

## Usage

```bash
# View the arm URDF (default)
ros2 launch urdf_viewer view_urdf.launch.py

# View any other URDF file
ros2 launch urdf_viewer view_urdf.launch.py urdf_file:=/absolute/path/to/robot.urdf

# View a xacro file
ros2 launch urdf_viewer view_urdf.launch.py urdf_file:=/absolute/path/to/robot.urdf.xacro

# Without the GUI joint sliders
ros2 launch urdf_viewer view_urdf.launch.py gui:=false
```

## Updating the Arm URDF

The arm URDF is generated from the Onshape CAD model using [onshape-to-robot](https://onshape-to-robot.readthedocs.io/en/latest/getting_started.html). To update the URDF after CAD changes:

1. Make sure `onshape-to-robot` is installed and configured with your Onshape API keys (see the [getting started guide](https://onshape-to-robot.readthedocs.io/en/latest/getting_started.html))

2. Run the update script from anywhere in the workspace:
   ```bash
   bash src/arm/urdf_viewer/urdf/scripts/urdf_asset.sh
   ```

   This script will:
   - Run `onshape-to-robot` to pull the latest URDF and mesh assets from Onshape
   - Fix mesh paths in `robot.urdf` to use the correct ROS package paths (`package://urdf_viewer/urdf/assets/`)

3. Rebuild the package to install the updated files:
   ```bash
   colcon build --packages-select urdf_viewer
   source install/setup.bash
   ```

## Launch Arguments

| Argument    | Default                              | Description                              |
|-------------|--------------------------------------|------------------------------------------|
| `urdf_file` | `urdf_viewer/urdf/robot.urdf`       | Absolute path to `.urdf` or `.xacro` file |
| `gui`       | `true`                               | Launch joint slider GUI (`true`/`false`)  |
