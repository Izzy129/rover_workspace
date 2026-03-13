import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

import genesis as gs


JOINT_NAMES = ['base_slide', 'shoulder', 'elbow', 'wrist_twist', 'claw']

# Mapping from our 5 controllable joints to DOF indices in the MJCF.
# The MJCF has 6 DOFs: base_slide(0), shoulder(1), elbow(2),
# wrist_twist(3), claw(4), claw_right(5). claw_right is coupled via
# an equality constraint so we only control claw(4).
JOINT_DOF_INDICES = [0, 1, 2, 3, 4]


class GenesisBridgeNode(Node):
    def __init__(self):
        super().__init__('genesis_bridge_node')

        # Initialize Genesis
        gs.init(backend=gs.cuda)

        # Create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=0.01),
            show_viewer=True,
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(1.5, -1.0, 1.2),
                camera_lookat=(0.0, 0.0, 0.4),
                camera_fov=45,
                max_FPS=60,
            ),
        )

        # Add ground plane
        self.scene.add_entity(gs.morphs.Plane())

        # Load arm MJCF
        desc_dir = get_package_share_directory('genesis_arm_description')
        mjcf_path = desc_dir + '/mjcf/arm.xml'
        self.get_logger().info(f'Loading MJCF from: {mjcf_path}')

        self.arm = self.scene.add_entity(
            gs.morphs.MJCF(file=mjcf_path),
        )

        # Build the scene
        self.scene.build()

        # Set PD control gains for position control
        n_dofs = len(JOINT_DOF_INDICES)
        # Include claw_right DOF (index 5) in gains arrays since Genesis
        # expects gains for all DOFs
        total_dofs = 6  # base_slide, shoulder, elbow, wrist_twist, claw, claw_right
        kp = np.array([1000.0, 2000.0, 1500.0, 500.0, 200.0, 200.0])
        kv = np.array([100.0, 200.0, 150.0, 50.0, 20.0, 20.0])
        self.arm.set_dofs_kp(kp)
        self.arm.set_dofs_kv(kv)

        # Latest target (all zeros initially)
        self.target_positions = np.zeros(n_dofs)
        self.target_velocities = np.zeros(n_dofs)

        # Subscriber for target joints from controller
        self.sub_target = self.create_subscription(
            JointState, '/target_joints', self.target_callback, 10)

        # Publisher for joint state feedback
        self.pub_state = self.create_publisher(JointState, '/joint_states', 10)

        # Sim loop timer at 100 Hz
        self.timer = self.create_timer(0.01, self.sim_step)

        self.get_logger().info('Genesis bridge node started (100 Hz)')

    def target_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                idx = JOINT_NAMES.index(name)
                if idx < len(self.target_positions):
                    if i < len(msg.position):
                        self.target_positions[idx] = msg.position[i]
                    if i < len(msg.velocity):
                        self.target_velocities[idx] = msg.velocity[i]

    def sim_step(self):
        # Apply position targets via PD control
        # Build full 6-DOF target (claw_right mirrors claw via equality constraint)
        full_target = np.zeros(6)
        full_target[:5] = self.target_positions
        full_target[5] = self.target_positions[4]  # mirror claw
        self.arm.control_dofs_position(full_target)

        # Step simulation
        self.scene.step()

        # Read joint state
        positions = self.arm.get_dofs_position().cpu().numpy().flatten()
        velocities = self.arm.get_dofs_velocity().cpu().numpy().flatten()

        # Publish joint state (only the 5 controllable joints)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(positions[i]) for i in JOINT_DOF_INDICES]
        msg.velocity = [float(velocities[i]) for i in JOINT_DOF_INDICES]
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GenesisBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
