#! /usr/bin/env python3

import os
import threading
import http.server
import asyncio
import websockets
import json
import signal
import time
import sys
import importlib.util
from pathlib import Path

import rclpy
from openai import OpenAI

TASK_PROGRAM_HEADER = "def task_program():"
REPO_ROOT = Path(__file__).resolve().parent
INTERFACE_FILE = REPO_ROOT / "interface.html"
PROMPT_FILE = REPO_ROOT / "openai_chat_completion_prefix.py"

rclpy.init()

httpd = None
server_thread = None
ws_server = None


async def broadcast_json(clients, payload):
  disconnected = []
  for websocket in list(clients):
    try:
      await websocket.send(json.dumps(payload))
    except websockets.exceptions.ConnectionClosed:
      disconnected.append(websocket)
    except Exception:
      disconnected.append(websocket)
  for websocket in disconnected:
    clients.discard(websocket)


def make_status_callback(loop, clients):
  def status_callback(payload):
    message = {
      "type": "action_status",
      "action_status": payload,
    }
    try:
      asyncio.run_coroutine_threadsafe(broadcast_json(clients, message), loop)
    except RuntimeError:
      pass
  return status_callback


def load_openai_api_key() -> str:
  key_path = REPO_ROOT / ".openai_api_key"
  if key_path.exists():
    key = key_path.read_text().strip()
  else:
    key = os.getenv("OPENAI_API_KEY", "").strip()
  if not key:
    raise RuntimeError(
      "OpenAI API key not found. Create '.openai_api_key' in the "
      "codebotler repo or set OPENAI_API_KEY."
    )
  return key


def load_prompt_messages(prompt_path: Path) -> list[dict]:
  spec = importlib.util.spec_from_file_location("codebotler_prompt", prompt_path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module.messages


def ensure_task_program(code: str) -> str:
  code = code.strip()
  if code.startswith(TASK_PROGRAM_HEADER):
    return code
  return f"{TASK_PROGRAM_HEADER}\n{code}".strip()


class OpenAICodeGenerator:
  def __init__(self, model: str):
    print("Using OpenAI Chat model:", model)
    self.model = model
    self.client = OpenAI(api_key=load_openai_api_key())
    self.messages = load_prompt_messages(PROMPT_FILE)

  def generate_one(self, prompt, stop_sequences, temperature, top_p, max_tokens):
    if len(stop_sequences) > 4:
      raise ValueError("OpenAI API only supports up to 4 stop sequences.")

    response = self.client.chat.completions.create(
      model=self.model,
      messages=self.messages + [{"role": "user", "content": prompt}],
      stop=stop_sequences,
      temperature=temperature,
      top_p=top_p,
      max_completion_tokens=max_tokens,
    )
    usage = response.usage
    if usage is not None:
      print(f"Tokens used: input={usage.prompt_tokens}, output={usage.completion_tokens}")
    return ensure_task_program(response.choices[0].message.content or "")


def serve_interface_html(args):
  global httpd
  class HTMLFileHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
      self.send_response(200)
      self.send_header('Content-type', 'text/html')
      self.end_headers()
      with open(INTERFACE_FILE, 'r') as file:
        html = file.read()
        html = html.replace("ws://localhost:8190",
                            f"ws://{args.ip}:{args.ws_port}")
      self.wfile.write(bytes(html, 'utf8'))
  print(f"Starting server at http://{args.ip}:{args.port}")
  try:
    httpd = http.server.HTTPServer((args.ip, args.port), HTMLFileHandler)
    httpd.serve_forever()
  except Exception as e:
    print("HTTP server error: " + str(e))
    shutdown(None, None)

def generate_code(prompt, args, generator):
  start_time = time.time()
  stop_sequences = ["\n#", "\nclass", "```"]
  code = generator.generate_one(prompt=prompt,
                                stop_sequences=stop_sequences,
                                temperature=args.temperature,
                                top_p=args.top_p,
                                max_tokens=args.max_tokens)
  end_time = time.time()
  time_str = f"{round(end_time - start_time, 2)}"
  print(f"Code generation time: {time_str} seconds")
  return code, time_str

def execute(code, use_robot, status_callback=None):
  if code is None or not code.strip():
    print("No generated code available. Ignoring execute request.")
    if status_callback is not None:
      status_callback({
        "call": "None",
        "status": "IDLE",
        "detail": "No generated code available.",
        "timestamp": time.time(),
      })
    return
  print(f"Received execution request for program:\n```python\n{code}\n```")

  from robot_client import execute_task_program
  robot_execution_thread = threading.Thread(
    target=execute_task_program,
    name="robot_execute",
    args=[code, use_robot, status_callback])
  robot_execution_thread.start()

async def handle_message(websocket, message, args, generator, use_robot, status_callback):
  data = json.loads(message)
  if data['type'] == 'code':
    print("Received code generation request")
    code, time_str = generate_code(data['prompt'], args, generator)
    response = {"code": f"{code}", "timing": time_str}
    await websocket.send(json.dumps(response))
  elif data['type'] == 'execute':
    print("Executing generated code...")
    code = data.get('code', '')
    execute(code, use_robot, status_callback)
  else:
    print("Unknown message type: " + data['type'])

async def ws_main(websocket, _path, args, generator, use_robot, connected_clients, status_callback):
  ## Whenever a client connects to the websocket, ws_main is called,
  ## and websocket refers to the connection with that specific client.
  connected_clients.add(websocket)
  print(f"Client has connected; {len(connected_clients)} clients...")

  async def receive_messages():
    try:
      async for message in websocket:
        await handle_message(websocket, message, args, generator, use_robot, status_callback)
    except websockets.exceptions.ConnectionClosedOK:
      print("Client has disconnected...")
    except websockets.exceptions.ConnectionClosedError as e:
      print(f"Client has disconnected with error: {e}")
    finally:
      connected_clients.discard(websocket)
      print(f"A client has been removed; {len(connected_clients)} clients...")

  await receive_messages()

async def start_ws_server(args, generator, use_robot):
  global ws_server
  connected_clients = set()
  loop = asyncio.get_running_loop()
  status_callback = make_status_callback(loop, connected_clients)
  ## Start the websocket server
  ws_server = await websockets.serve(
    lambda ws, path="": ws_main(
      ws, path, args, generator, use_robot, connected_clients, status_callback),
    args.ip, args.ws_port)
  print(f"WebSocket server started at ws://{args.ip}:{args.ws_port}")

  await asyncio.gather(ws_server.wait_closed())

def run_ws_server(args, generator, use_robot):
  try:
    asyncio.run(start_ws_server(args, generator, use_robot))
  except Exception as e:
    print("Websocket error: " + str(e))
    shutdown(None, None)

def shutdown(sig, frame):
  global server_thread, httpd, ws_server
  print(" Shutting down server.")
  try:
    rclpy.shutdown()
  except RuntimeError:
    pass
  if httpd is not None:
    httpd.server_close()
    httpd.shutdown()
  if server_thread is not None and threading.current_thread() != server_thread:
    server_thread.join()
  try:
    running_loop = asyncio.get_running_loop()
    print(" Cancelling asyncio tasks")
    for task in asyncio.all_tasks(loop=running_loop):
      task.cancel()
    print(" Requested stopping asyncio loop")
    running_loop.stop()
  except RuntimeError:
    pass
  if ws_server is not None:
    ws_server.close()
  if sig == signal.SIGINT or sig == signal.SIGTERM:
    exit_code = 0
  else:
    exit_code = 1
  sys.exit(exit_code)

def main():
  global server_thread
  import argparse
  parser = argparse.ArgumentParser()

  parser.add_argument('--ip', type=str, help='IP address', default="localhost")
  parser.add_argument('--port', type=int, help='HTML server port number', default=8080)
  parser.add_argument('--ws-port', type=int, help='Websocket server port number', default=8190)
  parser.add_argument('--model-name', type=str, help='Model name', default='gpt-5.4-mini')
  parser.add_argument("--max-tokens", type=int, default=512)
  parser.add_argument("--top-p", type=float, default=0.95)
  parser.add_argument("--temperature", type=float, default=0.2)
  parser.add_argument('--robot', action='store_true', help='Use real robot action clients instead of simulated DSL calls')
  args = parser.parse_args()

  signal.signal(signal.SIGINT, shutdown)

  generator = OpenAICodeGenerator(model=args.model_name)
  server_thread = threading.Thread(target=serve_interface_html,
                                   name="HTTP server thread",
                                   args=[args])
  server_thread.start()

  run_ws_server(args, generator, use_robot=args.robot)

if __name__ == "__main__":
  main()
