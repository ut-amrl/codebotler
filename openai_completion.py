import os
import openai
import sys

openai.api_key = os.getenv("OPENAI_API_KEY")

# If there are no arguments, use a default prompt
task_prompt = "Go to Arjun's office, ask him if he is ready to meet, and come back and tell me what he said."
if len(sys.argv) > 1:
    task_prompt = sys.argv[1]

print("Task Prompt:")
print(task_prompt)

# Load the prompt from the file "chatgpt_prompt.md"
with open("chatgpt_prompt.md", "r") as f:
    pre_prompt = f.read()

# Append task_prompt to the pre_prompt in quotes.
prompt = pre_prompt + f" \"{task_prompt}\"" + "\nProgram:\n"
# print("Prompt:")
# print("=============================================")
# print(prompt)
# print("=============================================")


print("Getting completion from OpenAI API...")
response = openai.Completion.create(
  model="text-davinci-003",
  prompt=prompt,
  temperature=0.7,
  max_tokens=256,
  top_p=1,
  frequency_penalty=0,
  presence_penalty=0
)

print("Response from OpenAI API:")
print("=============================================")
print(response.choices[0].text)
print("=============================================")