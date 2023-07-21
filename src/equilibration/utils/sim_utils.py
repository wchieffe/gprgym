import os
import subprocess


def launch_sim():
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scripts\launch_sim.py')
    process = subprocess.Popen(
        ["python", path],
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
