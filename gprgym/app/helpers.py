import os
import subprocess


def launch_sim():
    process = subprocess.Popen(
        ["python", "sim\launch_sim.py"],
        start_new_session=True
    )
    with open("isaac_sim.pid", "w") as f:
        f.write(str(process.pid))


def kill_sim():
    try:
        with open("isaac_sim.pid", "r") as f:
            pid = int(f.read())
        process = subprocess.Popen(["kill", "-TERM", str(pid)])
        process.communicate()
        os.remove("isaac_sim.pid")
    except Exception as e:
        pass
