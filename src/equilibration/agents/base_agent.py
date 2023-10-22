from dotenv import load_dotenv
from langchain.agents import AgentType, initialize_agent
from langchain.llms import OpenAI

import zmq

from utils.skill_utils import load_skills

load_dotenv()


class BaseAgent:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5558")
        self.update_skills()


    def update_skills(self):
        # Build array of langchain tools  based on the skills defined throughout /skills directory
        self.tools = []
        self.all_skills = load_skills()
        for _, skill_class in self.all_skills.items():
            self.tools.append(skill_class.as_tool(self.socket))

        # Initialize the LLM agent using those tools
        llm = OpenAI(temperature=0)
        print("tools: ", self.tools)
        self.agent = initialize_agent(self.tools, llm, agent=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION, verbose=True)


    def user_input(self, prompt: str):
        # request = f"You are controlling a franka robot arm. {prompt}"
        response = self.agent.run(prompt)
        return response
