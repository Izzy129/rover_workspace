# Arm Description

This package contains the URDF description and launch files for the robotic arm.

## Controlling the Arm

### Launch the Arm Simulation

First, launch the arm simulation:

```bash
ros2 launch arm_description arm_sim.launch.py
```

### Send Joint Trajectory Commands

To control the arm joints, publish to the joint trajectory topic:

```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
  joint_names: ['base_sliding_joint', 'shoulder_joint', 'elbow_joint'],
  points: [{
    positions: [0.1, 0.8, -1.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}" -1
```

This command will move:
- `base_sliding_joint` to position 0.1
- `shoulder_joint` to position 0.8
- `elbow_joint` to position -1.0

The movement will complete in 2 seconds.
