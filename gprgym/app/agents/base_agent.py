from dotenv import load_dotenv
from langchain.agents import AgentType, initialize_agent
from langchain.llms import OpenAI

from app.utils.skills import load_skills

load_dotenv()

class BaseAgent:
    def __init__(self):
        # Build array of langchain tools  based on the skills defined throughout /skills directory
        self.tools = []
        skills = load_skills()
        for skill in skills:
            self.tools.append(skill.as_tool())

        # Initialize the LLM agent using those tools
        llm = OpenAI(temperature=0)
        self.agent = initialize_agent(self.tools, llm, agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION, verbose=True)


    def user_input(self, prompt):
        response = self.agent.run(prompt)
        return response
