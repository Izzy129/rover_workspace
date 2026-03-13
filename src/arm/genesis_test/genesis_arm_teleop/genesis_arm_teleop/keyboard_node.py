import sys
import termios
import threading

import evdev
from evdev import InputDevice, categorize, ecodes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ['base_slide', 'shoulder', 'elbow', 'wrist_twist', 'claw']

# evdev key codes -> (joint_index, direction)
KEY_BINDINGS = {
    ecodes.KEY_A: (0, -1.0),   # base_slide left
    ecodes.KEY_D: (0, 1.0),    # base_slide right
    ecodes.KEY_W: (1, 1.0),    # shoulder up
    ecodes.KEY_S: (1, -1.0),   # shoulder down
    ecodes.KEY_Q: (2, 1.0),    # elbow up
    ecodes.KEY_E: (2, -1.0),   # elbow down
    ecodes.KEY_R: (3, 1.0),    # wrist_twist CW
    ecodes.KEY_F: (3, -1.0),   # wrist_twist CCW
    ecodes.KEY_O: (4, 1.0),    # claw open
    ecodes.KEY_C: (4, -1.0),   # claw close
}

# Velocity step sizes per joint
STEP_SIZES = {
    0: 0.05,   # prismatic (m/s)
    1: 0.5,    # revolute (rad/s)
    2: 0.5,    # revolute (rad/s)
    3: 1.0,    # revolute (rad/s)
    4: 0.5,    # revolute (rad/s)
}


def find_keyboard():
    """Find the first keyboard device in /dev/input/."""
    for path in evdev.list_devices():
        dev = InputDevice(path)
        caps = dev.capabilities(verbose=False)
        # EV_KEY = 1; check if it has typical keyboard keys
        if ecodes.EV_KEY in caps:
            key_codes = caps[ecodes.EV_KEY]
            if ecodes.KEY_A in key_codes and ecodes.KEY_Z in key_codes:
                return dev
    return None


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.pub = self.create_publisher(JointState, '/cmd_joint_vels', 10)

        # Set of currently pressed key codes
        self.pressed_keys = set()
        self.lock = threading.Lock()

        # Disable terminal echo so keypresses don't print,
        # but do NOT grab the device (keyboard still works elsewhere).
        self._old_termios = None
        if sys.stdin.isatty():
            fd = sys.stdin.fileno()
            self._old_termios = termios.tcgetattr(fd)
            new = termios.tcgetattr(fd)
            new[3] &= ~termios.ECHO  # disable echo
            termios.tcsetattr(fd, termios.TCSANOW, new)

        # Find keyboard
        self.device = find_keyboard()
        if self.device is None:
            self.get_logger().error(
                'No keyboard found! Make sure you are in the "input" group:\n'
                '  sudo usermod -aG input $USER\n'
                'Then log out and back in.')
            raise RuntimeError('No keyboard device found')

        self.get_logger().info(f'Using keyboard: {self.device.name} ({self.device.path})')

        # Start evdev reader thread
        self.reader_thread = threading.Thread(target=self._read_events, daemon=True)
        self.reader_thread.start()

        # Publish timer at 20 Hz
        self.pub_timer = self.create_timer(0.05, self.publish_velocities)

        self.get_logger().info(
            'Keyboard teleop started\n'
            '  a/d : base slide left/right\n'
            '  w/s : shoulder up/down\n'
            '  q/e : elbow up/down\n'
            '  r/f : wrist twist CW/CCW\n'
            '  o/c : claw open/close\n'
            '  SPACE : emergency stop\n'
            '  Ctrl+C : quit')

    def _read_events(self):
        """Background thread: read raw key press/release events."""
        try:
            for event in self.device.read_loop():
                if event.type != ecodes.EV_KEY:
                    continue

                key_event = categorize(event)

                with self.lock:
                    if key_event.keystate == key_event.key_down:
                        if event.code == ecodes.KEY_SPACE:
                            self.pressed_keys.clear()
                        elif event.code in KEY_BINDINGS:
                            self.pressed_keys.add(event.code)

                    elif key_event.keystate == key_event.key_up:
                        self.pressed_keys.discard(event.code)

        except OSError:
            self.get_logger().warn('Keyboard device disconnected')

    def publish_velocities(self):
        velocities = [0.0] * len(JOINT_NAMES)

        with self.lock:
            for key_code in self.pressed_keys:
                if key_code in KEY_BINDINGS:
                    joint_idx, direction = KEY_BINDINGS[key_code]
                    step = STEP_SIZES[joint_idx]
                    velocities[joint_idx] += direction * step

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.velocity = velocities
        self.pub.publish(msg)

    def destroy_node(self):
        # Restore terminal echo
        if self._old_termios is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._old_termios)
            except (OSError, IOError):
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info('Shutting down keyboard teleop')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
