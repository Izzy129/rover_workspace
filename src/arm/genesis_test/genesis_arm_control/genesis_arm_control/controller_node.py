import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


JOINT_NAMES = ['base_slide', 'shoulder', 'elbow', 'wrist_twist', 'claw']


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Declare parameters for each joint
        for name in JOINT_NAMES:
            prefix = f'joints.{name}'
            self.declare_parameter(f'{prefix}.type', 'revolute')
            self.declare_parameter(f'{prefix}.min_position', -3.14)
            self.declare_parameter(f'{prefix}.max_position', 3.14)
            self.declare_parameter(f'{prefix}.max_velocity', 1.0)
            self.declare_parameter(f'{prefix}.max_acceleration', 2.0)

        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('velocity_smoothing', 0.8)

        # Load joint limits
        self.joint_limits = {}
        for name in JOINT_NAMES:
            prefix = f'joints.{name}'
            self.joint_limits[name] = {
                'type': self.get_parameter(f'{prefix}.type').value,
                'min_pos': self.get_parameter(f'{prefix}.min_position').value,
                'max_pos': self.get_parameter(f'{prefix}.max_position').value,
                'max_vel': self.get_parameter(f'{prefix}.max_velocity').value,
                'max_acc': self.get_parameter(f'{prefix}.max_acceleration').value,
            }

        control_rate = self.get_parameter('control_rate').value
        self.alpha = self.get_parameter('velocity_smoothing').value
        self.dt = 1.0 / control_rate

        # State
        self.target_positions = {name: 0.0 for name in JOINT_NAMES}
        self.smoothed_velocities = {name: 0.0 for name in JOINT_NAMES}
        self.cmd_velocities = {name: 0.0 for name in JOINT_NAMES}
        self.current_positions = {name: 0.0 for name in JOINT_NAMES}

        # Control mode: 'teleop' or 'autonomous'
        self.control_mode = 'teleop'

        # Subscribers
        self.sub_cmd = self.create_subscription(
            JointState, '/cmd_joint_vels', self.cmd_vel_callback, 10)
        self.sub_cmd_auto = self.create_subscription(
            JointState, '/cmd_joint_vels_auto', self.cmd_vel_auto_callback, 10)
        self.sub_state = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)

        # Publisher
        self.pub_target = self.create_publisher(JointState, '/target_joints', 10)

        # Service to switch control mode
        self.srv_mode = self.create_service(
            SetBool, '/set_control_mode', self.set_mode_callback)

        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'Arm controller started at {control_rate} Hz, '
            f'smoothing alpha={self.alpha}')

    def cmd_vel_callback(self, msg: JointState):
        if self.control_mode != 'teleop':
            return
        self._update_cmd_from_msg(msg)

    def cmd_vel_auto_callback(self, msg: JointState):
        if self.control_mode != 'autonomous':
            return
        self._update_cmd_from_msg(msg)

    def _update_cmd_from_msg(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES and i < len(msg.velocity):
                self.cmd_velocities[name] = msg.velocity[i]

    def state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES and i < len(msg.position):
                self.current_positions[name] = msg.position[i]

    def set_mode_callback(self, request, response):
        if request.data:
            self.control_mode = 'autonomous'
            response.message = 'Switched to autonomous mode'
        else:
            self.control_mode = 'teleop'
            response.message = 'Switched to teleop mode'
        response.success = True
        self.get_logger().info(response.message)
        return response

    def control_loop(self):
        positions = []
        velocities = []

        for name in JOINT_NAMES:
            limits = self.joint_limits[name]
            cmd_vel = self.cmd_velocities[name]

            # Exponential velocity smoothing
            smoothed = (self.alpha * cmd_vel +
                        (1.0 - self.alpha) * self.smoothed_velocities[name])

            # Clamp velocity
            max_vel = limits['max_vel']
            smoothed = max(-max_vel, min(max_vel, smoothed))

            self.smoothed_velocities[name] = smoothed

            # Integrate to get target position
            self.target_positions[name] += smoothed * self.dt

            # Clamp position
            self.target_positions[name] = max(
                limits['min_pos'],
                min(limits['max_pos'], self.target_positions[name]))

            positions.append(self.target_positions[name])
            velocities.append(smoothed)

        # Publish target
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        msg.velocity = velocities
        self.pub_target.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
