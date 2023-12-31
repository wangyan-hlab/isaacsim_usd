import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # TODO: chamge the follow paths
        robot_description_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/scripts/rmpflow/robot_descriptor.yaml"
        rmpflow_config_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/scripts/rmpflow/fr_rmpflow_common.yaml"
        ## for robotiq85
        # urdf_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/urdf/FR5_V6_RTQ85.urdf"
        # end_effector_frame_name = "robotiq_85_base_link"
        ## for wsg50
        # urdf_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/urdf/FR5_V6_WSG50.urdf"
        # end_effector_frame_name = "base"
        ## for dahuan_pgi80
        urdf_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/urdf/FR5_V6_DHPGI80.urdf"
        end_effector_frame_name = "base"
        self.rmpflow = mg.lula.motion_policies.RmpFlow(robot_description_path=robot_description_path,
                                                        rmpflow_config_path=rmpflow_config_path,
                                                        urdf_path=urdf_path,
                                                        end_effector_frame_name=end_effector_frame_name,
                                                        maximum_substep_size=0.00334)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )