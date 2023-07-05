from fastapi import FastAPI
import uvicorn

from helpers import is_simulation_running, launch_simulation, stop_simulation_process
    
app = FastAPI()

@app.get("/start-simulation")
def start_simulation():
    if is_simulation_running():
        return {"message": "Simulation is already running."}
    pid = launch_simulation()
    return {"message": "Simulation started.", "pid": pid}

@app.get("/stop-simulation")
def stop_simulation():
    if not is_simulation_running():
        return {"message": "Simulation is not running."}
    stop_simulation_process()
    return {"message": "Simulation stopped."}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)