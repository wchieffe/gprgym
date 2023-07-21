import numpy as np
from pydantic import BaseModel, Field
from typing import List

try:
    # TODO: Better way to ignore import errors when agent is loading the skill
    from omni.isaac.franka.controllers import RMPFlowController
except:
    pass

from skills import BaseSkill


class Coordinates(BaseModel):
    x: float = Field()
    y: float = Field()
    z: float = Field()


class MoveToPositionArgsSchema(BaseModel):
    waypoints: List[Coordinates]


class MoveToPositionSkill(BaseSkill):
    def  __init__(self):
        self.description = "Move the robot's end effector to a series of waypoints."
        self.args_schema = MoveToPositionArgsSchema
        super().__init__()


    def execute(self, args, scene):
        controller = RMPFlowController(
            name = "rmp_flow_controller",
            robot_articulation = scene.franka,
        )
        articulation_controller = scene.franka.get_articulation_controller()

        waypoints = np.array(args.waypoints)
        for i in range(len(waypoints)):
            gripper_target_position = waypoints[i]
            while True:
                gripper_current_position = scene.franka.gripper.get_world_pose()[0]
                diff = np.sum(np.abs(gripper_current_position - gripper_target_position))
                if diff < 0.1:
                    break
                actions = controller.forward(
                    target_end_effector_position = gripper_target_position
                )
                articulation_controller.apply_action(actions)
                self.world.step(render=True)
