from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
import os

my_world = World(stage_units_in_meters=1.0)
#TODO: change this to your own path
asset_path = "/home/yan/Documents/isaacsim_usd/FR5_V6/usd/FR5_V6_DHPGI80/FR5_V6_DHPGI80_table.usd"
# asset_path = os.path.join(os.getcwd(), "FR5_V6/usd/FR5_V6_DHPGI80/FR5_V6_DHPGI80_table.usd")
prim_path = "/World/FR5_V6"
add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
#define the gripper
gripper = ParallelGripper(
    #We chose the following values while inspecting the articulation
    end_effector_prim_path=prim_path+"/robot/base",
    joint_prim_names=["left_driver_joint", "right_driver_joint"],
    joint_opened_positions=np.array([0.0, 0.0]),
    joint_closed_positions=np.array([-0.04, -0.04]),
    action_deltas=np.array([0.04, 0.04]),
)
#define the manipulator
my_fr = my_world.scene.add(SingleManipulator(prim_path=prim_path+"/robot", 
                                             name="fr5_v6_robot",
                                             end_effector_prim_name="base", 
                                             gripper=gripper))
#set the default positions of the other gripper joints to be opened so
#that its out of the way of the joints we want to control when gripping an object for instance.
joints_default_positions = np.zeros(8)
joints_default_positions[1] = -1.57
joints_default_positions[2] = 1.57
joints_default_positions[3] = -1.57
joints_default_positions[4] = -1.57
joints_default_positions[6] = 0
joints_default_positions[7] = 0
my_fr.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
        gripper_positions = my_fr.gripper.get_joint_positions()
        if i < 100:
            #close the gripper slowly
            my_fr.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.0004, gripper_positions[1] - 0.0004]))
        if i > 100:
            #open the gripper slowly
            my_fr.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.0004, gripper_positions[1] + 0.0004]))
        if i == 200:
            i = 0

simulation_app.close()