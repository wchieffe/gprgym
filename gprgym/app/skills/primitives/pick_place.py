import numpy as np

from omni.isaac.franka.controllers import PickPlaceController

from app.skills import BaseSkill

class PickPlace(BaseSkill):
    def  __init__(self):
        super().__init__()
        self.description = "Pick up the cube and place it in another location another ."

    def execute(self, world, franka, fancy_cube):
        """Pick up the box and move it somewhere else."""
        controller = PickPlaceController(
            name="pick_place_controller",
            gripper=franka.gripper,
            robot_articulation=self.franka,
        )
        while True:
            cube_position, _ = self.fancy_cube.get_world_pose()
            goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
            current_joint_positions = self.franka.get_joint_positions()
            actions = controller.forward(
                picking_position=cube_position,
                placing_position=goal_position,
                current_joint_positions=current_joint_positions,
            )
            self.franka.apply_action(actions)
            # Only for the pick and place controller, indicating if the state
            # machine reached the final state.
            if controller.is_done():
                self.world.pause()
                break
            self.world.step(render=True) 
