import os
import openai

openai.api_key = os.getenv("OPENAI_API_KEY")

response = openai.Completion.create(
  model="text-davinci-003",
  prompt="provide a list of x,y,z coordinates along a path that makes a franka robot arm look like it's waving",
  temperature=0.4,
  max_tokens=64,
  top_p=1,
  frequency_penalty=0,
  presence_penalty=0
)