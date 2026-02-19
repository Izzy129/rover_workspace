#!/usr/bin/env python3
"""
Gazebo bridge node: translates /arm_joint_commands to /arm_vel_controller/commands.

This replaces sim_driver.py in the Gazebo pipeline. It receives JointState
commands from keyboard_teleop and forwards them to the ros2_control velocity
controller that drives the Gazebo physics simulation.

- Velocity mode (msg.velocity populated): forwards velocities directly
- Position mode (msg.position populated): runs a P-controller to compute
  velocity commands toward the target position based on current joint states
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')

        # P-controller gain for position mode
        self.declare_parameter('position_kp', 3.0)

        # Per-joint velocity caps (match sim driver limits)
        self.declare_parameter('slider_max_vel', 0.05)
        self.declare_parameter('shoulder_max_vel', 0.67)
        self.declare_parameter('elbow_max_vel', 0.67)

        self.kp = self.get_parameter('position_kp').value
        self.max_vels = [
            self.get_parameter('slider_max_vel').value,
            self.get_parameter('shoulder_max_vel').value,
            self.get_parameter('elbow_max_vel').value,
        ]

        # Joint order must match arm_controllers.yaml
        self.joint_names = ['slider', 'shoulder_1', 'elbow_1']
        self.current_positions = [0.0, 0.0, 0.0]

        self.last_command: JointState | None = None

        # Subscribe to teleop commands
        self.cmd_sub = self.create_subscription(
            JointState, '/arm_joint_commands', self.command_callback, 10)

        # Subscribe to joint states for P-controller feedback
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)

        # Publish velocity commands to the ros2_control ForwardCommandController
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/arm_vel_controller/commands', 10)

        # Publish at 20Hz (matches teleop rate)
        self.timer = self.create_timer(0.05, self.publish_commands)

    def state_callback(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]

    def command_callback(self, msg: JointState):
        self.last_command = msg

    def publish_commands(self):
        velocities = [0.0, 0.0, 0.0]

        if self.last_command is not None:
            msg = self.last_command

            if msg.velocity:
                # Velocity mode: forward velocity commands directly
                name_to_vel = dict(zip(msg.name, msg.velocity))
                for i, name in enumerate(self.joint_names):
                    velocities[i] = name_to_vel.get(name, 0.0)

            elif msg.position:
                # Position mode: P-controller toward target position
                name_to_pos = dict(zip(msg.name, msg.position))
                for i, name in enumerate(self.joint_names):
                    if name in name_to_pos:
                        error = name_to_pos[name] - self.current_positions[i]
                        vel = self.kp * error
                        vel = max(-self.max_vels[i], min(self.max_vels[i], vel))
                        velocities[i] = vel

        out = Float64MultiArray()
        out.data = velocities
        self.vel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
