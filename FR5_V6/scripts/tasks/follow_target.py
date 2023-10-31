from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np


# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "fr_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change this to the robot usd file.
        asset_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/urdf/FR5_V6_gripper_frame/FR5_V6_gripper_frame.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/FR5_V6")
        gripper = ParallelGripper(
            end_effector_prim_path="/World/FR5_V6/robotiq_85_base_link",
            joint_prim_names=["robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.804, -0.804]),
            action_deltas=np.array([-0.804, 0.804]))
        manipulator = SingleManipulator(prim_path="/World/FR5_V6",
                                        name="fr5",
                                        end_effector_prim_name="robotiq_85_base_link",
                                        gripper=gripper)
        joints_default_positions = np.zeros(12)
        joints_default_positions[1] = -1.57
        joints_default_positions[2] = 1.57
        joints_default_positions[6] = 0
        joints_default_positions[8] = 0
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator