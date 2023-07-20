import numpy as np

from omni.isaac.franka.controllers import RMPFlowController

from app.skills import BaseSkill

class MoveToPosition(BaseSkill):
    def  __init__(self, world, franka):
        self.world = world
        self.robot = franka

    def execute(self):
        """Move the franka's gripper to a series of (x, y, z) waypoints in the shape of a wave"""
        controller = RMPFlowController(
            name="rmp_flow_controller",
            robot_articulation=self.franka,
        )
        articulation_controller = self.franka.get_articulation_controller()
        waypoints = np.array([
            [0.5, 0.3, 0.2],
            [0.5, 0.4, 0.2],
            [0.5, 0.5, 0.2],
            [0.5, 0.6, 0.2],
            [0.5, 0.7, 0.2],
            [0.5, 0.8, 0.2],
            [0.5, 0.9, 0.2],
            [0.5, 1.0, 0.2],
            [0.5, 0.9, 0.2],
            [0.5, 0.8, 0.2],
            [0.5, 0.7, 0.2],
            [0.5, 0.6, 0.2],
            [0.5, 0.5, 0.2],
            [0.5, 0.4, 0.2],
            [0.5, 0.3, 0.2]
        ])
        for i in range(len(waypoints)):
            gripper_target_position = waypoints[i]
            while True:
                gripper_current_position = self.franka.gripper.get_world_pose()[0]
                diff = np.sum(np.abs(gripper_current_position - gripper_target_position))
                print("current position: ", gripper_current_position)
                print("target position: ", gripper_target_position)
                print("diff: ", diff)
                if diff < 0.1:
                    break
                actions = controller.forward(
                    target_end_effector_position = gripper_target_position
                )
                articulation_controller.apply_action(actions)
                self.world.step(render=True)
