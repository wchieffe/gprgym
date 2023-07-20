from fastapi import FastAPI
import logging
import uvicorn
import zmq

from app.agents import BaseAgent
from app.utils.sim import launch_sim, kill_sim
from app.utils.skills import load_skills

app = FastAPI()
agent = BaseAgent()
context = zmq.Context()
socket = None


@app.on_event("startup")
async def startup_event():
    logging.info("Starting simulation")
    launch_sim()
    global socket
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5558")


@app.on_event("shutdown")
async def shutdown_event():
    kill_sim()
    logging.info("Shutting down simulation")
    socket.close()
    context.term()


@app.post("/user_input")
def update_item(request: str):
    response = agent.user_input(request)
    return {"Response: ": response}


# For debugging purposes, create dedicated endpoint for each skill
for skill_name, skill_class in load_skills().items():
    @app.get("/" + skill_name)
    def send_command(args: skill_class.args_schema):
        payload = {"skill_name": skill_name, "args": args}
        socket.send_json(payload)
        message = socket.recv()
        return {"result": message}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
