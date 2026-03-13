"""Analytical IK for prismatic-base + 2R planar arm.

NOT IMPLEMENTED YET. This is a placeholder for Phase 6.
The arm's geometry (prismatic base + shoulder + elbow) yields
a closed-form 2-link planar IK solution. Wrist twist and claw
are set independently.
"""


class ArmKinematics:
    def forward(self, joint_positions: dict) -> dict:
        """Compute end-effector pose from joint positions."""
        raise NotImplementedError("Implement in Phase 6")

    def inverse(self, target_pose: dict) -> dict:
        """Compute joint positions from target end-effector pose."""
        raise NotImplementedError("Implement in Phase 6")
