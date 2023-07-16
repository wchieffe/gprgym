# Launch Isaac Sim before any other imports
# Default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import signal
import time
import zmq

from app.skills.basic_scene import setup_scene
from app.skills.move_to import move_to_position
from app.skills.pick_place import pick_place

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

        if message == b"pick":
            pick_place(world, franka, fancy_cube)
        elif message == b"move to":
            move_to_position(world, franka)
        else:
            break

        response = b"Success!"
        socket.send(response)

        time.sleep(1)

    simulation_app.close()