#!/usr/bin/env python3
import sys
import tty
import termios
import select
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('arm_keyboard_teleop')

        # Declare parameters
        self.declare_parameter('joint_names', ['slider', 'shoulder_1', 'elbow_1'])
        self.declare_parameter('slider_step', 0.005)
        self.declare_parameter('shoulder_step', 0.03)
        self.declare_parameter('elbow_step', 0.03)
        self.declare_parameter('slider_limits', [-0.25, 0.0])
        self.declare_parameter('shoulder_limits', [-3.14159, 3.14159])
        self.declare_parameter('elbow_limits', [-3.14159, 3.14159])
        self.declare_parameter('publish_rate', 20.0)

        # Read parameters
        self.joint_names = self.get_parameter('joint_names').value
        self.slider_step = self.get_parameter('slider_step').value
        self.shoulder_step = self.get_parameter('shoulder_step').value
        self.elbow_step = self.get_parameter('elbow_step').value
        self.slider_limits = self.get_parameter('slider_limits').value
        self.shoulder_limits = self.get_parameter('shoulder_limits').value
        self.elbow_limits = self.get_parameter('elbow_limits').value
        publish_rate = self.get_parameter('publish_rate').value

        # Joint limits as list of [min, max] per joint
        self.limits = [
            self.slider_limits,
            self.shoulder_limits,
            self.elbow_limits,
        ]

        # Step sizes per joint index
        self.steps = [self.slider_step, self.shoulder_step, self.elbow_step]

        # Current desired positions
        self.positions = [0.0, 0.0, 0.0]

        # Active deltas set by keyboard thread
        self.deltas = [0.0, 0.0, 0.0]
        self.lock = threading.Lock()

        # Publisher
        self.cmd_pub = self.create_publisher(JointState, '/arm_joint_commands', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

        # Key mapping: key -> (joint_index, direction)
        self.key_map = {
            'w': (0, 1.0),   # slider up
            's': (0, -1.0),  # slider down
            'a': (1, 1.0),   # shoulder left
            'd': (1, -1.0),  # shoulder right
            'j': (2, 1.0),   # elbow left
            'l': (2, -1.0),  # elbow right
        }

        # Track running state
        self.running = True

        # Print instructions before raw mode takes over the terminal
        self.print_instructions()

        # Start keyboard thread
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    def print_instructions(self):
        print('---------------------------')
        print('  Arm Keyboard Teleop')
        print('---------------------------')
        print('  w/s : slider up/down')
        print('  a/d : shoulder left/right')
        print('  j/l : elbow left/right')
        print('  0   : zero all joints')
        print('  q   : quit')
        print('---------------------------')
        sys.stdout.flush()

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error(
                'No TTY detected. Run with: ros2 run arm_teleop keyboard_teleop.py')
            return
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                # Poll stdin with 50ms timeout
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key(key)
                else:
                    # No key pressed, clear deltas
                    with self.lock:
                        self.deltas = [0.0, 0.0, 0.0]
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        if key == 'q':
            self.running = False
            rclpy.shutdown()
            return

        if key == '0':
            with self.lock:
                self.positions = [0.0, 0.0, 0.0]
                self.deltas = [0.0, 0.0, 0.0]
            sys.stdout.write('\r\nZeroed all joints\r\n')
            sys.stdout.flush()
            return

        if key in self.key_map:
            joint_idx, direction = self.key_map[key]
            with self.lock:
                self.deltas = [0.0, 0.0, 0.0]
                self.deltas[joint_idx] = direction * self.steps[joint_idx]

    def publish_command(self):
        with self.lock:
            # Apply deltas
            for i in range(3):
                self.positions[i] += self.deltas[i]
                # Clamp to limits
                self.positions[i] = max(self.limits[i][0],
                                        min(self.limits[i][1], self.positions[i]))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.positions)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
