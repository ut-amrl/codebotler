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
from code_generation.completions import AutoModel, PaLMModel, OpenAIModel, TextGenerationModel
import threading

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
prompt_prefix = ""
prompt_suffix = ""

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
  httpd = http.server.HTTPServer((args.ip, args.port), HTMLFileHandler)
  httpd.serve_forever()

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
  global prompt_prefix
  global prompt_suffix
  global code_timeout
  start_time = time.time()
  prompt = prompt_prefix + prompt + prompt_suffix
  stop_sequences = ["#", "\ndef ", "\nclass", "import "]
  code = model.generate_one(prompt=prompt,
                            stop_sequences=stop_sequences,
                            temperature=0.9,
                            top_p=0.99999,
                            max_tokens=512)
  end_time = time.time()
  print(f"Code generation time: {round(end_time - start_time, 2)} seconds")
  code = (prompt_suffix + code).strip()
  return code

def execute(code):
  global ros_available
  global robot_available
  global robot_interface
  if not ros_available:
    print("ROS not available. Ignoring execute request.")
  elif not robot_available:
    print("Robot not available. Ignoring execute request.")
  else:
    from robot_interface.src.interface import execute_task_program
    robot_execution_thread = threading.Thread(target=execute_task_program, name="robot_execute", args=[code, robot_interface])
    robot_execution_thread.start()

async def handle_message(websocket, message):
  data = json.loads(message)
  if data['type'] == 'code':
    print("Received code generation request")
    code = await generate_code(data['prompt'])
    response = {"code": f"{code}"}
    await websocket.send(json.dumps(response))
    if data['execute']:
      print("Executing generated code")
      execute(code)
  elif data['type'] == 'eval':
    print("Received eval request")
    # await eval(websocket, data)
  elif data['type'] == 'execute':
    print("Received execute request")
    execute(data['code'])
    await websocket.close()
  else:
    print("Unknown message type: " + data['type'])

async def ws_main(websocket, path):
  try:
    async for message in websocket:
      await handle_message(websocket, message)
  except websockets.exceptions.ConnectionClosed:
    pass

def start_completion_callback(args):
  global asyncio_loop, ws_server
  # Create an asyncio event loop
  asyncio_loop = asyncio.new_event_loop()
  asyncio.set_event_loop(asyncio_loop)
  start_server = websockets.serve(ws_main, args.ip, args.ws_port)

  try:
    ws_server = asyncio_loop.run_until_complete(start_server)
    asyncio_loop.run_forever()
  except Exception as e:
    print("Websocket error: " + str(e))
    shutdown(None, None)

def shutdown(sig, frame):
  global ros_available, robot_available, robot_interface, server_thread, asyncio_loop, httpd, ws_server
  print(" Shutting down Server")
  if ros_available:
    rospy.signal_shutdown("Shutting down Server")
  if robot_available and ros_available:
    robot_interface._cancel_goals()
  httpd.server_close()
  httpd.shutdown()
  server_thread.join()
  for task in asyncio.all_tasks(loop=asyncio_loop):
    task.cancel()
  asyncio_loop.stop()
  ws_server.close()
  sys.exit(0)

def main():
  global server_thread
  global prompt_prefix
  global prompt_suffix
  global ros_available
  global robot_available
  global robot_interface
  global code_timeout
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument('--ip', type=str, help='IP address', default="localhost")
  parser.add_argument('--port', type=int, help='HTML server port number', default=8080)
  parser.add_argument('--ws-port', type=int, help='Websocket server port number', default=8190)
  parser.add_argument("--model-type", choices=["openai", "palm", "automodel", "hf-textgen"], default="openai")
  parser.add_argument('--model-name', type=str, help='Model name', default='text-davinci-003')
  parser.add_argument('--prompt-prefix', type=Path, help='Prompt prefix', default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=Path, help='Prompt suffix', default='code_generation/prompt_suffix.py')
  parser.add_argument('--interface-page', type=Path, help='Interface page', default='code_generation/interface.html')
  parser.add_argument('--max-workers', type=int, help='Maximum number of workers', default=1)
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
    from robot_interface.src.interface import RobotInterface
    robot_interface = RobotInterface()

  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  load_model(args)
  server_thread = threading.Thread(target=serve_interface_html, args=[args])
  server_thread.start()

  start_completion_callback(args)

if __name__ == "__main__":
  main()