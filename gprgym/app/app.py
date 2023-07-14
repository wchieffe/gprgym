from fastapi import FastAPI
import logging
import uvicorn
import zmq

from helpers import launch_sim, kill_sim

app = FastAPI()
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


@app.get("/pick")
def send_pick_command():
    socket.send(b"pick")
    message = socket.recv()
    print(f"Received response: {message}")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
