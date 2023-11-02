from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import numpy as np
from tasks.pick_place_table_rtq85 import PickPlace
from controllers.pick_place import PickPlaceController

my_world = World(stage_units_in_meters=1.0)


target_position = np.array([0, -0.3, 0])
target_position[2] = 0.0515 / 2.0
cube_initial_orientation =  np.array([0, -0.4, 0])
my_task = PickPlace(name="fr_pick_place_table", 
                    cube_initial_position=cube_initial_orientation,
                    target_position=target_position)

my_world.add_task(my_task)
my_world.reset()
my_fr = my_world.scene.get_object("fr5_v6_robot")
#initialize the controller
my_controller = PickPlaceController(name="controller", robot_articulation=my_fr, gripper=my_fr.gripper)
task_params = my_world.get_task("fr_pick_place_table").get_params()
articulation_controller = my_fr.get_articulation_controller()
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        #forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            # This offset needs tuning as well
            end_effector_offset=np.array([0, 0, 0.145]),
            end_effector_orientation=np.array([0, -1, 0, 1])
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
simulation_app.close()