import os
import subprocess


def launch_sim():
    python_path = "C:\\Users\ov-user\AppData\Local\ov\pkg\isaac_sim-2022.2.1\python.exe" # TODO: fix this
    script_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scripts\launch_sim.py')
    process = subprocess.Popen(
        [python_path, script_path],
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
