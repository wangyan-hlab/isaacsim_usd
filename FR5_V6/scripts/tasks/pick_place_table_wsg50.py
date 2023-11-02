from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np


class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "fr_pick_place_table_wsg50",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change the asset path here
        asset_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/usd/FR5_V6_WSG50/FR5_V6_WSG50_table.usd"
        prim_path = "/World/FR5_V6"
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
        gripper = ParallelGripper(
            end_effector_prim_path=prim_path+"/robot/base",
            joint_prim_names=["left_driver_joint", "right_driver_joint"],
            joint_opened_positions=np.array([0.054, 0.054]),
            joint_closed_positions=np.array([0.0, 0.0]),
            action_deltas=np.array([0.03, 0.03]) )
        manipulator = SingleManipulator(prim_path=prim_path+"/robot",
                                        name="fr5_v6_robot",
                                        end_effector_prim_name="base",
                                        gripper=gripper)
        joints_default_positions = np.zeros(8)
        joints_default_positions[1] = -1.57
        joints_default_positions[2] = 1.57
        joints_default_positions[3] = -1.57
        joints_default_positions[4] = -1.57
        joints_default_positions[6] = 0
        joints_default_positions[7] = 0
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator