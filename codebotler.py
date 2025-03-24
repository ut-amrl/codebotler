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
import sys
from pathlib import Path
import threading
from vllm import SamplingParams

from roboeval.misc.llm_generation_utils import post_process_vllm_generation, post_process_program
from roboeval.models.model_factory import load_model
from roboeval.misc.utils import load_module

ros_available = False
robot_available = False
robot_interface = None
try:
    import rospy
    ros_available = True
    rospy.init_node('ros_interface', anonymous=False)
except:
    print("Could not import rospy. Robot interface is not available.")
    ros_available = False

httpd = None
server_thread = None
model = None
asyncio_loop = None
ws_server = None

def serve_interface_html(args):
  global httpd
  class HTMLFileHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
      self.send_response(200)
      self.send_header('Content-type', 'text/html')
      self.end_headers()
      with open(args.interface_page, 'r') as file:
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

def build_prompts(args, prompt):
  if args.model_type == "openai":
    messages = load_module("", args.chat_prompt_prefix).__dict__["messages"]
    for msg in messages:
        if msg["role"] == "user":
            msg["content"] = "# Instruction: " + msg["content"]
    messages += [{"role": "user", "content": "# Instruction: " + prompt}]
    prompt = messages
  else:
    prefix = Path(args.prompt_prefix).read_text()
    suffix = Path(args.prompt_suffix).read_text()
    prompt = prefix + prompt + suffix 
  return [prompt]

def generate_code(prompt, args):
  global model, code_timeout
  stop_params = {"stop": ["\n#", "\ndef", "```", "import"]}
  prompts = build_prompts(args, prompt)
  start_time = time.time()
  
  if args.model_type == "vllm":
      sampling_params = SamplingParams(
          temperature=args.temperature,
          max_tokens=args.max_tokens,
          top_p=args.top_p,
          **stop_params
      )
      outputs = model.generate(prompts, sampling_params)
      programs = post_process_vllm_generation(outputs)
  elif args.model_type == "gemini":
      unprocessed_programs = model.generate(prompts, **stop_params, temperature=args.temperature, top_p=args.top_p, max_tokens=args.max_tokens)
      programs = []
      for program in unprocessed_programs:
          program = post_process_program(program)
          programs.append(program)
  elif args.model_type == "openai":
      programs = model.generate(prompts, **stop_params, temperature=args.temperature, top_p=args.top_p, max_tokens=args.max_tokens)
  else:
      raise ValueError(f"To Be Implemented: {args.model_type}")
  end_time = time.time()
  print(f"Code generation time: {round(end_time - start_time, 2)} seconds")

  return programs[0]

def execute(code):
  global ros_available
  global robot_available
  global robot_interface
  if not ros_available:
    print("ROS not available. Ignoring execute request.")
  elif not robot_available:
    print("Robot not available. Ignoring execute request.")
  else:
    from codebotler_robot_interface.src.robot_client_interface import execute_task_program
    robot_execution_thread = threading.Thread(target=execute_task_program, name="robot_execute", args=[code, robot_interface])
    robot_execution_thread.start()

async def handle_message(websocket, message, args):
  data = json.loads(message)
  if data['type'] == 'code':
    print("Received code generation request")
    code = generate_code(data['prompt'], args)
    response = {"code": f"{code}"}
    await websocket.send(json.dumps(response))
    if data['execute']:
      print("Executing generated code")
      execute(code)
  elif data['type'] == 'eval':
    print("Received eval request")
    # await eval(websocket, data)
  elif data['type'] == 'execute':
    print("Executing generated code")
    execute(data['code'])
    await websocket.close()
  else:
    print("Unknown message type: " + data['type'])

async def ws_main(websocket, args):
  try:
    async for message in websocket:
      await handle_message(websocket, message, args)
  except websockets.exceptions.ConnectionClosed:
    pass

def start_completion_callback(args):
    global asyncio_loop, ws_server

    async def run_server():
        global ws_server

        async def handler(ws):
            await ws_main(ws, args)  # args is accessible from the outer scope

        ws_server = await websockets.serve(handler, args.ip, args.ws_port)
        print(f"WebSocket server started on {args.ip}:{args.ws_port}")
        await ws_server.wait_closed()

    # Create and set the asyncio event loop
    asyncio_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(asyncio_loop)

    try:
        asyncio_loop.run_until_complete(run_server())
    except Exception as e:
        print("Websocket error: " + str(e))
        shutdown(None, None)

def shutdown(sig, frame):
  global ros_available, robot_available, robot_interface, server_thread, asyncio_loop, httpd, ws_server
  print(" Shutting down server.")
  if robot_available and ros_available and robot_interface is not None:
    robot_interface._cancel_goals()
    print("Waiting for 2s to preempt robot actions...")
    time.sleep(2)
  if ros_available:
    rospy.signal_shutdown("Shutting down Server")
  if httpd is not None:
    httpd.server_close()
    httpd.shutdown()
  if server_thread is not None and threading.current_thread() != server_thread:
    server_thread.join()
  if asyncio_loop is not None:
    for task in asyncio.all_tasks(loop=asyncio_loop):
      task.cancel()
    asyncio_loop.stop()
  if ws_server is not None:
    ws_server.close()
  if sig == signal.SIGINT or sig == signal.SIGTERM:
    exit_code = 0
  else:
    exit_code = 1
  sys.exit(exit_code)

def main():
  global server_thread
  global ros_available
  global robot_available
  global robot_interface
  global code_timeout
  global model
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument('--ip', type=str, help='IP address', default="localhost")
  parser.add_argument('--port', type=int, help='HTML server port number', default=8080)
  parser.add_argument('--ws-port', type=int, help='Websocket server port number', default=8190)
  
  parser.add_argument("-mt", "--model_type", choices=["vllm", "openai", "gemini"], default="openai")
  parser.add_argument("-m", "--model_name_or_path", type=str, default="gpt-4")
  parser.add_argument("-temp", "--temperature", type=float, default=0.2)
  parser.add_argument("--max_tokens", type=int, default=512)
  parser.add_argument("--top_p", type=float, default=0.95)
  parser.add_argument("-tps", "--tensor_parallel_size", type=int, default=1)
  parser.add_argument("-gmu", "--gpu_memory_utilization", type=float, default=0.7)
  parser.add_argument("--use_llama3_inst", action="store_true") # apply llama 3 instruction template
    
  parser.add_argument('--chat-prompt-prefix', type=str, help='Prompt prefix for GPT chat completion only', default='roboeval/code_generation/openai_chat_completion_prefix.py')
  parser.add_argument('--prompt-prefix', type=str, help='Prompt prefix for all but GPT chat completion', default='roboeval/code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=str, help='Prompt suffix for all but GPT chat completion', default='roboeval/code_generation/prompt_suffix.py')
  parser.add_argument('--interface-page', type=Path, help='Interface page', default='roboeval/code_generation/interface.html')
    
  parser.add_argument('--robot', action='store_true', help='Flag to indicate if the robot is available')
  parser.add_argument('--timeout', type=int, help='Code generation timeout in seconds', default=20)

  if ros_available:
    args = parser.parse_args(rospy.myargv()[1:])
  else:
    args = parser.parse_args()

  robot_available = args.robot
  code_timeout = args.timeout

  signal.signal(signal.SIGINT, shutdown)

  if robot_available and ros_available:
    from codebotler_robot_interface.src.robot_client_interface import RobotInterface
    robot_interface = RobotInterface()

  model = load_model(args)
  server_thread = threading.Thread(target=serve_interface_html,
                                   name="HTTP server thread",
                                   args=[args])
  server_thread.start()

  start_completion_callback(args)

if __name__ == "__main__":
  main()
