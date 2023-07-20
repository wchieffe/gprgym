from fastapi import FastAPI
import logging
import uvicorn


from app.agents import BaseAgent
from app.utils.sim_utils import launch_sim, kill_sim

app = FastAPI()
agent = BaseAgent()


@app.on_event("startup")
async def startup_event():
    logging.info("Starting simulation")
    launch_sim()


@app.on_event("shutdown")
async def shutdown_event():
    kill_sim()
    logging.info("Shutting down simulation")
    agent.socket.close()
    agent.context.term()


@app.post("/user_input")
def update_item(request: str):
    response = agent.user_input(request)
    return {"Response: ": response}


# For debugging purposes, create dedicated endpoint for each skill
# TODO: How to update when agent.all_skills updates
for skill_name, skill_class in agent.all_skills.items():
    @app.get("/" + skill_name)
    def send_command(args: skill_class.args_schema = None):
        payload = {"skill_name": skill_name, "args": args}
        agent.socket.send_json(payload)
        message = agent.socket.recv()
        return {"result": message}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
