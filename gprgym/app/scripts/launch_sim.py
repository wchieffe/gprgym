# Default first two lines for any standalone application in Isaac Sim
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import signal
import zmq

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka import Franka

from app.skills import PickPlace, MoveToPosition


def setup_scene():
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


if __name__ == "__main__":
    world, franka, fancy_cube = setup_scene()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5558")

    while True:
        print("Waiting for event")
        message = socket.recv()
        print(f"Received request: {message}")

        # TODO: Feed in custom context object with everything I might need to pass in? 
        # Then I don't need to manually add each new skill
        if message == b"PickPlace":
            PickPlace.execute(world, franka, fancy_cube)
        elif message == b"MoveToPosition":
            MoveToPosition.execute(world, franka)
        else:
            break

        response = b"Success!"
        socket.send(response)

    simulation_app.close()