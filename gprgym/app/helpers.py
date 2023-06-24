import os
import signal
import subprocess
import time

def is_simulation_running():
    # Check if the Isaac Sim process is running
    return os.path.exists("isaac_sim.pid")

def launch_simulation():
    process = subprocess.Popen(
        ["python", "move_franka.py"],
        start_new_session=True
    )
    with open("isaac_sim.pid", "w") as f:
        f.write(str(process.pid))


def stop_simulation_process():
    with open("isaac_sim.pid", "r") as f:
        pid = int(f.read())
    # try:
    #     process = subprocess.Popen(["kill", "-TERM", str(pid)])
    #     process.communicate()
    # except subprocess.CalledProcessError:
    #     pass
    os.remove("isaac_sim.pid")
