#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmSimDriver(Node):
    """Simulated arm driver with trapezoidal velocity profiles.

    Supports two control modes (auto-detected from incoming messages):
    - Position mode: msg.position populated -> move to target with trapezoidal profile
    - Velocity mode: msg.velocity populated -> track desired velocity with acceleration limits

    Replace this with a real hardware driver when integrating with the
    physical arm.
    """

    def __init__(self):
        super().__init__('arm_sim_driver')

        # Declare motion profile parameters
        self.declare_parameter('slider_max_vel', 0.05)
        self.declare_parameter('slider_max_accel', 0.1)
        self.declare_parameter('shoulder_max_vel', 1.0)
        self.declare_parameter('shoulder_max_accel', 2.0)
        self.declare_parameter('elbow_max_vel', 1.0)
        self.declare_parameter('elbow_max_accel', 2.0)
        self.declare_parameter('slider_limits', [-0.25, 0.0])
        self.declare_parameter('shoulder_limits', [-3.14159, 3.14159])
        self.declare_parameter('elbow_limits', [-3.14159, 3.14159])
        self.declare_parameter('wrist_max_vel', 1.0)
        self.declare_parameter('wrist_max_accel', 2.0)
        self.declare_parameter('claw_max_vel', 0.5)
        self.declare_parameter('claw_max_accel', 1.0)
        self.declare_parameter('wrist_limits', [-3.14159, 3.14159])
        self.declare_parameter('claw_limits', [0.0, 1.065])

        self.max_vel = [
            self.get_parameter('slider_max_vel').value,
            self.get_parameter('shoulder_max_vel').value,
            self.get_parameter('elbow_max_vel').value,
            self.get_parameter('wrist_max_vel').value,
            self.get_parameter('claw_max_vel').value,
        ]
        self.max_accel = [
            self.get_parameter('slider_max_accel').value,
            self.get_parameter('shoulder_max_accel').value,
            self.get_parameter('elbow_max_accel').value,
            self.get_parameter('wrist_max_accel').value,
            self.get_parameter('claw_max_accel').value,
        ]
        self.limits = [
            self.get_parameter('slider_limits').value,
            self.get_parameter('shoulder_limits').value,
            self.get_parameter('elbow_limits').value,
            self.get_parameter('wrist_limits').value,
            self.get_parameter('claw_limits').value,
        ]

        self.joint_names = ['slider', 'shoulder_1', 'elbow_1', 'wrist', 'claw']
        self.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Position mode state
        self.targets = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Velocity mode state
        self.desired_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Control mode: 'position' or 'velocity'
        self.control_mode = 'position'

        self.cmd_sub = self.create_subscription(
            JointState, '/arm_joint_commands', self.command_callback, 10)

        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_and_publish)

    def command_callback(self, msg):
        self.joint_names = list(msg.name)
        if msg.velocity:
            self.control_mode = 'velocity'
            self.desired_velocities = list(msg.velocity)
        elif msg.position:
            self.control_mode = 'position'
            self.targets = list(msg.position)

    def update_and_publish(self):
        if self.control_mode == 'position':
            self._update_position_mode()
        else:
            self._update_velocity_mode()

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = self.joint_names
        state.position = list(self.positions)
        self.state_pub.publish(state)

    def _update_position_mode(self):
        for i in range(len(self.positions)):
            error = self.targets[i] - self.positions[i]
            distance = abs(error)

            if distance < 1e-4:
                self.positions[i] = self.targets[i]
                self.velocities[i] = 0.0
                continue

            direction = math.copysign(1.0, error)
            v = self.velocities[i]
            a_max = self.max_accel[i]
            v_max = self.max_vel[i]

            stopping_dist = (v * v) / (2.0 * a_max) if a_max > 0 else 0.0

            if stopping_dist >= distance:
                v -= direction * a_max * self.dt
            else:
                v += direction * a_max * self.dt
                if abs(v) > v_max:
                    v = direction * v_max

            self.velocities[i] = v
            self.positions[i] += v * self.dt

            # Prevent overshoot
            if (error > 0 and self.positions[i] > self.targets[i]) or \
               (error < 0 and self.positions[i] < self.targets[i]):
                self.positions[i] = self.targets[i]
                self.velocities[i] = 0.0

    def _update_velocity_mode(self):
        for i in range(len(self.positions)):
            desired_v = self.desired_velocities[i]
            v = self.velocities[i]
            a_max = self.max_accel[i]

            # Accelerate/decelerate toward desired velocity
            v_error = desired_v - v
            if abs(v_error) < a_max * self.dt:
                v = desired_v
            else:
                v += math.copysign(a_max * self.dt, v_error)

            # Clamp to max velocity
            v_max = self.max_vel[i]
            v = max(-v_max, min(v_max, v))

            self.velocities[i] = v
            self.positions[i] += v * self.dt

            # Clamp to joint limits
            lo, hi = self.limits[i]
            if self.positions[i] <= lo:
                self.positions[i] = lo
                self.velocities[i] = 0.0
            elif self.positions[i] >= hi:
                self.positions[i] = hi
                self.velocities[i] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ArmSimDriver()
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
