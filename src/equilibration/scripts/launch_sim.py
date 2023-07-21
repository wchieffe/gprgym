# Default first two lines for any standalone application in Isaac Sim
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import signal
import zmq

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka import Franka

from utils.skill_utils import load_skills


class Scene:
    def __init__(self):
        # Guideline: anything I might need to reference should use self
        self.world = World()
        self.world.scene.add_default_ground_plane()
        self.fancy_cube =  self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0.3, 0.3, 0.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )
        self.franka = self.world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        self.world.reset()
        self.franka.gripper.set_joint_positions(self.franka.gripper.joint_opened_positions)
        self.world.step(render=True) 


if __name__ == "__main__":
    scene = Scene()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5558")

    all_skills = load_skills()

    while True:
        print("Waiting for event")
        payload = socket.recv_json()

        # Execute skill specified in the zmq payload
        skill_class = all_skills[payload["skill_name"]]
        skill_class.execute(payload["args"], scene)

        response = b"Success!"
        socket.send(response)

    simulation_app.close()
