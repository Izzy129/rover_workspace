# lidar_publisher

Launch orchestration for the Livox MID-360 LiDAR, converting its PointCloud2 output to a LaserScan for the navigation subsystem.

**IMPORTANT NOTE: this package uses Livox's `livox_ros_driver2` (see [here](https://github.com/Livox-SDK/livox_ros_driver2)) and Livox SDK 2 (see [here](https://github.com/Livox-SDK/Livox-SDK2/)). Both of these are added as git submodules, so they will get cloned when you clone `rover_workspace`.**

# Before running, you'll need to install the Livox SDK 2 systemwide, by following [these instructions](https://github.com/Livox-SDK/Livox-SDK2#2-installation)

## **AUTO NAV NOTE (hi rocky): you'll want to subscribe to the `/lidar/scan` topic!!**
## Pipeline

```
livox_ros_driver2 → /livox/lidar (PointCloud2) → pointcloud_to_laserscan → /lidar/scan (LaserScan)
```

## Nodes (launched, not owned)

### livox_ros_driver2_node
Livox MID-360 driver. Publishes point cloud data from the LiDAR.

**Published Topics:**
- `/livox/lidar` (sensor_msgs/PointCloud2) - 3D point cloud at 10 Hz

**Parameters:**
- `xfer_format` (int, default: 0) - 0 for PointCloud2 format
- `multi_topic` (int, default: 0) - 0 for all LiDARs on one topic
- `data_src` (int, default: 0) - 0 for live LiDAR
- `publish_freq` (double, default: 10.0) - Publish rate in Hz
- `output_data_type` (int, default: 0) - Output format
- `frame_id` (string, default: "livox_frame") - TF frame ID (TODO: someone edit this with robot urdf TF stuff)
- `user_config_path` (string) - Path to MID360_config.json (in src/vision/lidar_publisher/config/MID360_config.json)

### pointcloud_to_laserscan_node
Converts 3D point cloud to 2D laser scan by slicing a horizontal band. 

*Note: Livox driver publishes in PointCloud2, but Nav needs LaserScan. This node handles the conversion.*

**Published Topics:**
- `/lidar/scan` (sensor_msgs/LaserScan) - 2D laser scan at 10 Hz

**Subscribed Topics:**
- `/livox/lidar` (sensor_msgs/PointCloud2)

**Parameters** (see `config/laserscan_params.yaml`):
- `min_height` / `max_height` - Vertical slice bounds (meters)
- `angle_min` / `angle_max` - Scan angular range (radians, default: full 360°)
- `angle_increment` - Angular resolution (~1°)
- `range_min` / `range_max` - Valid range bounds (0.1–40.0 m)
- `target_frame` - TF frame for the scan output

## Launch

```bash
ros2 launch lidar_publisher lidar_publisher.launch.py
```

## Configuration

- `config/MID360_config.json` - Livox network config (sensor IP: 192.168.1.100, host IP: 192.168.1.50)
 - see bottom of this [google doc](https://docs.google.com/document/d/1SHosyefHxz-XiZO1UowHAnpAiPquvEaUPYTM2lIaOl0/edit?tab=t.0) for ip information
- `config/laserscan_params.yaml` - pointcloud_to_laserscan tuning parameters

## Building livox_ros_driver2

The Livox driver requires a special build process. Use the included script:

```bash
./src/vision/lidar_publisher/build_livox_driver.sh
```

This handles the ROS2-specific setup (package.xml, launch files, cmake args). Livox-SDK2 must be installed system-wide (`liblivox_lidar_sdk_shared.so` in `/usr/local/lib`).

## Dependencies

- livox_ros_driver2
- pointcloud_to_laserscan (`sudo apt install ros-jazzy-pointcloud-to-laserscan`)
