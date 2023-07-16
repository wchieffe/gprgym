from dotenv import load_dotenv
import openai
import os
import importlib

load_dotenv()


class BaseAgent:
    def __init__(self):
        openai.api_key = os.getenv("OPENAI_API_KEY")

        # Populate skills attribute based on objects defined throughout /skills directory
        skills_path = os.path.join(os.path.dirname(__file__), 'skills')
        skill_files = [file for file in os.listdir(skills_path) if file.endswith('.py')]
        self.skills = []
        for skill_file in skill_files:
            module_name = f'skills.{os.path.splitext(skill_file)[0]}'
            module = importlib.import_module(module_name)
            skill_class = getattr(module, os.path.splitext(skill_file)[0].capitalize())
            skill_object = skill_class()
            self.skills.append(skill_object)


    def user_prompt(self, prompt):
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {
                "role": "user",
                "content": prompt
                }
            ],
            temperature=0.5,
            max_tokens=64,
            top_p=1.0,
            frequency_penalty=0.0,
            presence_penalty=0.0
        )
        return response
