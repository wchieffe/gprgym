#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

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
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()
controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

def pick_place():
    try:
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
    finally:
        simulation_app.close() # close Isaac Sim

import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
    print("loop")
    world.step(render=True) 

    #  Wait for next request from client
    message = socket.recv()
    print(f"Received request: {message}")

    if message == b"pick":
        pick_place()
    elif message == b"exit":
        break

    time.sleep(1)
