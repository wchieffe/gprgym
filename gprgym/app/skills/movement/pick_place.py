import numpy as np

from omni.isaac.franka.controllers import PickPlaceController
from app.skills import BaseSkill

class PickPlace(BaseSkill):
    def  __init__():
        pass

    def fire(world, franka, fancy_cube):
        controller = PickPlaceController(
            name="pick_place_controller",
            gripper=franka.gripper,
            robot_articulation=franka,
        )
        while True:
            cube_position, _ = fancy_cube.get_world_pose()
            goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
            current_joint_positions = franka.get_joint_positions()
            actions = controller.forward(
                picking_position=cube_position,
                placing_position=goal_position,
                current_joint_positions=current_joint_positions,
            )
            franka.apply_action(actions)
            # Only for the pick and place controller, indicating if the state
            # machine reached the final state.
            if controller.is_done():
                world.pause()
                break
            world.step(render=True) 
