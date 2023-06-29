#! /usr/bin/env python3

import os
import threading
import http.server
import socketserver
import asyncio
import websockets
import json
import signal
import time
from completions import AutoModel, PaLMModel, OpenAIModel, TextGenerationModel
import threading

model = None
prompt_prefix = ""
prompt_suffix = ""

def load_model(args):
  global model
  if args.model_type == "openai":
    # If there exists a ".openai_api_key" file, use that as the API key.
    if os.path.exists(".openai_api_key"):
      with open(".openai_api_key", "r") as f:
        openai_api_key = f.read().strip()
    else:
      openai_api_key = os.getenv("OPENAI_API_KEY")
    assert len(openai_api_key) > 0, \
        "OpenAI API key not found. " + \
        "Either create a '.openai_api_key' file or " + \
        "set the OPENAI_API_KEY environment variable."
    model = OpenAIModel(model=args.model_name, api_key = openai_api_key)
  elif args.model_type == "palm":
    # If there exists a ".palm_api_key" file, use that as the API key.
    if os.path.exists(".palm_api_key"):
      with open(".palm_api_key", "r") as f:
        palm_api_key = f.read().strip()
    else:
      palm_api_key = os.getenv("PALM_API_KEY")
    assert len(palm_api_key) > 0, \
        "PaLM API key not found. " + \
        "Either create a '.palm_api_key' file or " + \
        "set the PALM_API_KEY environment variable."
    model = PaLMModel(model=args.model_name, api_key = palm_api_key)
  elif args.model_type == "automodel":
    model = AutoModel(batch_size=1, path=args.model_name)
  elif args.model_type == "hf-textgen":
    model = TextGenerationModel(args.model_name, args.max_workers)
  else:
    raise ValueError(f"Unknown model type: {args.model_type}")

def generate_code(prompt):
  global model
  start_time = time.time()
  prompt = prompt_prefix + prompt + prompt_suffix
  stop_sequences = ["#", "\ndef ", "\nclass", "import "]
  code = model.generate_one(prompt=prompt,
                            stop_sequences=stop_sequences,
                            temperature=0.9,
                            top_p=0.99999,
                            max_tokens=512)
  end_time = time.time()
  code = (prompt_suffix + code).strip()
  return code

def main():
  global prompt_prefix
  global prompt_suffix
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument("--model-type", choices=["openai", "palm", "automodel", "hf-textgen"], default="openai")
  parser.add_argument('--model-name', type=str, help='Model name', default='text-davinci-003')
  parser.add_argument('--prompt-prefix', type=Path, help='Prompt prefix', default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=Path, help='Prompt suffix', default='code_generation/prompt_suffix.py')
  parser.add_argument('--max-workers', type=int, help='Maximum number of workers', default=1)
  parser.add_argument('--n', type=int, help='Number of completions', default=20)

  args = parser.parse_known_args()
  # Get all unparsed arguments.
  prompt = " ".join(args[1:][0])
  args = args[0]
  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  load_model(args)

  definitions = """
def get_current_location():
    ...
def get_all_rooms():
    ...
def go_to(room):
    ...
def is_in_room(item):
    ...
def say(text):
    ...
def ask(person, question, options):
    ...
  """
  print(definitions)
  print(f"Prompt=\"{prompt}\"")

  for i in range(args.n):
    code = generate_code(prompt)
    print(f"# {i+1}=======================================")
    print(code)
    print(f"# {i+1}=======================================")


if __name__ == "__main__":
  main()