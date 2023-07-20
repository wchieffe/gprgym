from dotenv import load_dotenv
from langchain.agents import AgentType, initialize_agent
from langchain.llms import OpenAI

from app.utils.skill_utils import load_skills

load_dotenv()

class BaseAgent:
    def __init__(self):
        # Build array of langchain tools  based on the skills defined throughout /skills directory
        self.tools = []
        for _, skill_class in load_skills().items():
            self.tools.append(skill_class.as_tool())

        # Initialize the LLM agent using those tools
        llm = OpenAI(temperature=0)
        self.agent = initialize_agent(self.tools, llm, agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION, verbose=True)


    def user_input(self, prompt):
        # TODO: Format prompt ("this is what the scene looks like. you are controlling a franka robot."). 
        # No need to add tools to it; langchain does that
        response = self.agent.run(prompt)
        return response
