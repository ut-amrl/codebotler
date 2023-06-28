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
from code_generation.completions import AutoModel, PaLMModel, OpenAIModel, TextGenerationModel
from robot_interface.src.interface import execute_task_program
import threading 

ros_available = False
robot_available = False
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
prompt_prefix = ""
prompt_suffix = ""

def serve_interface_html(args):
  global httpd
  html_file = args.interface_page
  class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass
  class HTMLFileHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
      self.send_response(200)
      self.send_header('Content-type', 'text/html')
      self.end_headers()
      with open(html_file, 'r') as file:
        html = file.read()
        html = html.replace("ws://localhost:8190",
                            f"ws://{args.ip}:{args.ws_port}")
      self.wfile.write(bytes(html, 'utf8'))
  print(f"Starting server at http://{args.ip}:{args.port}")
  with ThreadedHTTPServer((args.ip, args.port), HTMLFileHandler) as httpd:
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
  
async def handle_message(websocket, message):
  global ros_available
  global robot_available
  data = json.loads(message)
  if data['type'] == 'code':
    print("Received code request")
    code = generate_code(data['prompt'])
    response = {"code": f"{code}"}
    await websocket.send(json.dumps(response))
  elif data['type'] == 'eval':
    print("Received eval request")
    # await eval(websocket, data)
  elif data['type'] == 'execute':
    print("Received execute request")
    if not ros_available:
      print("ROS not available. Ignoring execute request.")
    elif not robot_available:
      print("Robot not available. Ignoring execute request.")
    else:
      robot_execution_thread = threading.Thread(target=execute_task_program, name="robot_execute", args=[data['code']])
      robot_execution_thread.start()
  else:
    print("Unknown message type: " + data['type'])

async def ws_main(websocket, path):
  print(f"Client connected.")
  try:
    async for message in websocket:
      await handle_message(websocket, message)
  except websockets.exceptions.ConnectionClosed:
    print("Client disconnected.")

def start_completion_callback(args):
  # Create an asyncio event loop
  loop = asyncio.new_event_loop()
  asyncio.set_event_loop(loop)
  start_server = websockets.serve(ws_main, args.ip, args.ws_port)

  # Register a signal handler to stop the server when Ctrl-C is pressed
  loop.add_signal_handler(
      signal.SIGINT,
      lambda: (print("INFO: Shutting down Server"), loop.stop()))

  try:
    server = loop.run_until_complete(start_server)
    loop.run_forever()
  except Exception as e:
    print("ERROR_INFO: " + str(e))
  finally:
    print("Closing server")
    for task in asyncio.all_tasks(loop=loop):
        task.cancel()
    server.close()
    loop.run_until_complete(server.wait_closed())
    loop.close()

def main():
  global server_thread
  global prompt_prefix
  global prompt_suffix
  global ros_available
  global robot_available
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument('--ip', type=str, help='IP address', default="localhost")
  parser.add_argument('--port',
                      type=int, help='HTML server port number', default=8080)
  parser.add_argument('--ws-port',
                      type=int, help='Websocket server port number',
                      default=8190)
  parser.add_argument("--model-type",
                      choices=["openai", "palm", "automodel", "hf-textgen"],
                      default="openai")
  parser.add_argument('--model-name',
                      type=str, help='Model name', default='text-davinci-003')
  parser.add_argument('--prompt-prefix',
                      type=Path, help='Prompt prefix',
                      default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix',
                      type=Path, help='Prompt suffix',
                      default='code_generation/prompt_suffix.py')
  parser.add_argument('--interface-page',
                      type=Path, help='Interface page',
                      default='code_generation/interface.html')
  parser.add_argument('--max-workers',
                      type=int, help='Maximum number of workers',
                      default=1)
  parser.add_argument('--robot', action='store_true', help='Flag to indicate if the robot is available')

  if ros_available:
    args = parser.parse_args(rospy.myargv()[1:])
  else:
    args = parser.parse_args()
  
  robot_available = args.robot

  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  load_model(args)
  server_thread = threading.Thread(target=serve_interface_html, args=[args])
  server_thread.start()

  start_completion_callback(args)

if __name__ == "__main__":
  main()