#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmSimDriver(Node):
    """Passthrough node that echoes joint commands to joint states.

    Replace this with a real hardware driver when integrating with the
    physical arm. The hardware driver would subscribe to the same
    /arm_joint_commands topic, send commands to the motors, read encoder
    feedback, and publish actual positions to /joint_states.
    """

    def __init__(self):
        super().__init__('arm_sim_driver')

        self.joint_names = ['slider', 'shoulder_1', 'elbow_1']
        self.positions = [0.0, 0.0, 0.0]

        self.cmd_sub = self.create_subscription(
            JointState, '/arm_joint_commands', self.command_callback, 10)

        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publish joint states at 20Hz so robot_state_publisher always has TF
        self.timer = self.create_timer(0.05, self.publish_state)

    def command_callback(self, msg):
        self.joint_names = list(msg.name)
        self.positions = list(msg.position)

    def publish_state(self):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = self.joint_names
        state.position = self.positions
        self.state_pub.publish(state)


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
