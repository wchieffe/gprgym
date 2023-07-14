from fastapi import FastAPI
import uvicorn

from helpers import is_simulation_running, launch_simulation, stop_simulation_process

app = FastAPI()

@app.get("/start-simulation")
def start_simulation():
    # Check if the simulation is already running
    if is_simulation_running():
        return {"message": "Simulation is already running."}

    # Launch Isaac Sim in headless mode
    launch_simulation()

    return {"message": "Simulation started."}

@app.get("/stop-simulation")
def stop_simulation():
    # Check if the simulation is running
    if not is_simulation_running():
        return {"message": "Simulation is not running."}

    # Stop the Isaac Sim process
    stop_simulation_process()

    return {"message": "Simulation stopped."}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)