import os
import openai

openai.api_key = os.getenv("OPENAI_API_KEY")

response = openai.Completion.create(
  model="text-davinci-003",
  prompt="Write a tagline for an ice cream shop.",
  temperature=0.4,
  max_tokens=64,
  top_p=1,
  frequency_penalty=0,
  presence_penalty=0
)