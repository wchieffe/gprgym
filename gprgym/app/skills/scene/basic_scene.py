import numpy as np

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka import Franka

from app.skills import BaseSkill

class BasicScene(BaseSkill):
    def  __init__():
        pass

    def fire():
        world = World()
        world.scene.add_default_ground_plane()
        fancy_cube =  world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0.3, 0.3, 0.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )
        franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        world.reset()
        franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
        world.step(render=True) 
        return world, franka, fancy_cube
