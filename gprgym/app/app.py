from fastapi import FastAPI
import logging
from pydantic import BaseModel
import uvicorn
import zmq

from app.agents import BaseAgent
from utils.sim_functions import launch_sim, kill_sim

app = FastAPI()
agent = BaseAgent()
context = zmq.Context()
socket = None


class GptRequest(BaseModel):
    prompt: str


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


@app.get("/pick")
def send_pick_command():
    socket.send(b"pick")
    message = socket.recv()
    return {"result": message}


@app.post("/text_command")
def update_item(request: GptRequest):
    
    response = agent.user_prompt(request.prompt)
    return {"Response: ": response}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
