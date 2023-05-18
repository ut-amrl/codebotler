import os
import openai
import sys
import json

openai.api_key = os.getenv("OPENAI_API_KEY")

# If there are no arguments, use a default prompt
task_prompt = "Go to Arjun's office, ask him if he is ready to meet, and come back and tell me what he said."
if len(sys.argv) > 1:
    task_prompt = sys.argv[1]

# If the API key is not set, exit.
if openai.api_key is None:
    print("OpenAI API key is not set.")
    sys.exit(1)

print("Task Prompt:")
print(task_prompt)


# Load the prompt from the file "chatgpt_prompt.md"
with open("gpt-3.5-turbo-prompt.json", "r") as f:
    pre_prompt = f.read()
pre_prompt = json.loads(pre_prompt)
pre_prompt.append({"role": "user", "content": task_prompt})

print("Prompt:")
print("=============================================")
print(pre_prompt)
print("=============================================")

print("Getting completion from OpenAI API...")
response = openai.ChatCompletion.create(
  model="gpt-3.5-turbo",
  messages = pre_prompt
)

print("Response from OpenAI API:")
print("=============================================")
print(response)
print("=============================================")