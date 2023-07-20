import numpy as np
from pydantic import BaseModel, Field

from omni.isaac.franka.controllers import PickPlaceController

from equilibration.skills import BaseSkill


class PickPlaceArgsSchema(BaseModel):
    x: float = Field()
    y: float = Field()
    z: float = Field()


class PickPlace(BaseSkill):
    def  __init__(self):
        super().__init__()
        self.description = "Pick up the cube and place it in another location."

    def execute(self, args, scene):
        controller = PickPlaceController(
            name="pick_place_controller",
            gripper=scene.franka.gripper,
            robot_articulation=scene.franka,
        )
        while True:
            cube_position, _ = scene.fancy_cube.get_world_pose()
            goal_position = np.array([args.x, args.y, args.z])
            current_joint_positions = scene.franka.get_joint_positions()
            actions = controller.forward(
                picking_position=cube_position,
                placing_position=goal_position,
                current_joint_positions=current_joint_positions,
            )
            scene.franka.apply_action(actions)
            # Only for the pick and place controller, indicating if the state
            # machine reached the final state.
            if controller.is_done():
                scene.world.pause()
                break
            scene.world.step(render=True) 
