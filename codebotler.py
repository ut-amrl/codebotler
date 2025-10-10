#! /usr/bin/env python3

import os
import threading
import concurrent.futures
import http.server
import socketserver
import asyncio
import websockets
import json
import signal
import time
import sys

from models.model_factory import load_model
from models.OpenAIChatModel import OpenAIChatModel


import threading

ros_available = False
robot_available = False
robot_interface = None
rclpy_instance = None
try:
    import rclpy
    ros_available = True
    rclpy_instance = rclpy
    rclpy.init()
except:
    print("Could not import rclpy. Robot interface is not available.")
    ros_available = False

httpd = None
server_thread = None
model = None
asyncio_loop = None
ws_server = None
prompt_prefix = ""
prompt_suffix = ""
pipe_descriptor = None
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

def generate_code(prompt, args):
  global model, prompt_prefix, prompt_suffix, code_timeout
  start_time = time.time()
  stop_sequences = ["\n#", "\nclass", "```"]
  if args.model_type != "openai-chat":
    prompt = prompt_prefix + prompt + prompt_suffix
    stop_sequences += ["\ndef"]
  code = model.generate_one(prompt=prompt,
                            stop_sequences=stop_sequences,
                            temperature=args.temperature,
                            top_p=args.top_p,
                            max_tokens=args.max_tokens)
  end_time = time.time()
  time_str = f"{round(end_time - start_time, 2)}"
  print(f"Code generation time: {time_str} seconds")
  if type(model) is not OpenAIChatModel:
      code = (prompt_suffix + code).strip()
  elif not code.startswith(prompt_suffix.strip()):
      code = (prompt_suffix + "\n" + code).strip()
  return code, time_str

def execute(code):
  global ros_available
  global robot_available
  global robot_interface
  print(f"Recieved execution request for program:\n```python\n{code}\n```")
  if not ros_available:
    print("ROS not available. Ignoring execute request.")
  elif not robot_available:
    print("Robot not available. Ignoring execute request.")
  else:
    from robot_interface.src.robot_client_interface import execute_task_program
    robot_execution_thread = threading.Thread(target=execute_task_program, name="robot_execute", args=[code, robot_interface])
    robot_execution_thread.start()

async def handle_message(websocket, message, args):
  data = json.loads(message)
  if data['type'] == 'code':
    print("Received code generation request")
    code, time_str = generate_code(data['prompt'], args)
    response = {"code": f"{code}", "timing": time_str}
    await websocket.send(json.dumps(response))
    if data['execute']:
      print("Executing generated code")
      execute(code)
  elif data['type'] == 'eval':
    print("Received eval request")
    # await eval(websocket, data)
  elif data['type'] == 'execute':
    print("Executing generated code...")
    execute(data['code'])
  else:
    print("Unknown message type: " + data['type'])

async def post_transcript(websocket, args, data, instruction, executebool):
  ## Push the message to the client
  print(f"Posting this message to the client: {data}")
  response = {"transcript": f"{data}"}
  if instruction is not None:
    response["instruction"] = instruction
  if executebool:
    response["doexecute"] = "true"
  await websocket.send(json.dumps(response))

async def post_code(websocket, args, data, time_str):
  ## Push the message to the client
  print(f"Posting generated code to the client")
  response = {"code": f"{data}", "timing": time_str}
  await websocket.send(json.dumps(response))

async def read_from_pipe(fd, executor):
  print(f"Entered `read_from_pipe`")
  loop = asyncio.get_running_loop()
  try:
    with os.fdopen(fd, "r") as f:
      while True:
        print(f"Waiting to read a line from the pipe")
        line = await loop.run_in_executor(
          executor,
          f.readline
        )
        print(f"Read this line: {line}")
        ## Only send transcripts that are marked as final
        if line.startswith("<FINAL>:"):
          yield line[8:].rstrip("\n")
  except asyncio.CancelledError:
    print("read_from_pipe canceled, closing file")
    raise
  except OSError:
    print("Pipe already closed")
  finally:
    print("Closing pipe descriptor")
    try:
      os.close(fd)
    except OSError:
      print("Pipe is already closed")

class Accumulator:
  def __init__(self):
    self.start_word = "cobot"
    self.end_word = "done"
    self.start_detected = False
    self.end_detected = False
    self.recording = False
    self.parts = list()
    self.instruction = None

  async def execute_detected(self, text):
    text_lower = text.lower()
    return ("yes" in text_lower) and ("execute" in text_lower) and ("please" in text_lower)

  async def accumulate(self, text):
    ## Returns None if no instruction has been completely formed yet.
    ## Returns the instruction (string) if an instruction has been fully formed.
    ## Our recording assumes the following: if start is seen, keep recording until the first end is seen.
    ## Consider the following cases:
    ## Text contains start, no end
    ## Text contains end, no start
    ## Text contains start and end
    ## Text contains no start, no end
    text_lower = text.lower()
    self.start_detected = self.start_word in text_lower
    self.end_detected = self.end_word in text_lower

    if self.recording: ## we are recording the instruction
      if self.start_detected and not self.end_detected:
        ## We should continue recording
        self.instruction = None
        self.recording = True
        self.parts.append(text)
      elif not self.start_detected and self.end_detected:
        ## We are done recording
        self.instruction = None
        self.recording = False
        e_idx = text_lower.find(self.end_word)
        self.parts.append(text[:e_idx])
        self.instruction = "".join(self.parts)
        self.parts = list()
      elif not self.start_detected and not self.end_detected:
        ## Keep recording
        self.instruction = None
        self.recording = True
        self.parts.append(text)
      else: ## We need to see if start comes before end
        s_idx = text_lower.find(self.start_word)
        e_idx = text_lower.find(self.end_word)
        if s_idx < e_idx:
          self.instruction = None
          self.recording = False
          self.parts.append(text[:e_idx])
          self.instruction = "".join(self.parts)
          self.parts = list()
        else:
          self.instruction = None
          self.recording = False
          self.parts.append(text[:e_idx])
          self.instruction = "".join(self.parts)
          self.parts = list()
          self.parts.append(text[s_idx+len(self.start_word):])
    else: ## we are currently not recording instructions
      if self.start_detected and not self.end_detected:
        ## We should start recording
        self.instruction = None
        self.recording = True
        s_idx = text_lower.find(self.start_word)
        self.parts.append(text[s_idx+len(self.start_word):])
      elif not self.start_detected and self.end_detected:
        ## Ignore this text input
        self.instruction = None
        self.recording = False
      elif not self.start_detected and not self.end_detected:
        ## Ignore this text input
        self.instruction = None
        self.recording = False
      else: ## start and end detected, we need to see if start comes before end
        s_idx = text_lower.find(self.start_word)
        e_idx = text_lower.find(self.end_word)
        self.instruction = None
        if s_idx < e_idx:
          self.parts.append(text[s_idx+len(self.start_word):e_idx])
          self.instruction = "".join(self.parts)
          self.parts = list()
        self.recording = False

    return self.instruction

async def ws_main(websocket, path, args, accumulator, connected_clients):
  ## Whenever a client connects to the websocket, ws_main is called,
  ## and websocket refers to the connection with that specific client.
  connected_clients.add(websocket)
  print(f"Client has connected; {len(connected_clients)} clients...")

  async def receive_messages():
    try:
      async for message in websocket:
        await handle_message(websocket, message, args)
    except websockets.exceptions.ConnectionClosedOK:
      print("Client has disconnected...")
    except websockets.exceptions.ConnectionClosedError as e:
      print(f"Client has disconnected with error: {e}")
    finally:
      connected_clients.discard(websocket)
      print(f"A client has been removed; {len(connected_clients)} clients...")

  await receive_messages()
  #await asyncio.gather(receive_messages(), send_messages())

async def broadcast_transcripts_from_pipe(args, fd, accumulator, ws_server, clients, last_gen_code, lock, executor):
  try:
    async for line in read_from_pipe(fd, executor):
      instruction = await accumulator.accumulate(line)
      executebool = await accumulator.execute_detected(line)
      ## Start to execute the most recent program that was generated, if any
      if executebool:
        codetoexec = None
        async with lock:
          if last_gen_code[0] is not None:
            codetoexec = last_gen_code[0]
        if codetoexec is not None:
          execute(codetoexec)
      ## Stream the transcribed text from the pipe to all connected clients.
      client_copy = clients.copy()
      for websocket in client_copy:
        try:
          await post_transcript(websocket, args, line, instruction, executebool)
        except:
          pass
      ## If the transcription contains an instruction, generate a code plan for it
      ## and cache the code plan locally, and send it to the connected clients
      if instruction is not None and len(instruction) > 0:
        code, time_str = await asyncio.to_thread(generate_code, instruction, args)
        async with lock:
          last_gen_code[0] = code
        client_copy = clients.copy()
        for websocket in client_copy:
          try:
            await post_code(websocket, args, code, time_str)
          except:
            pass
  except asyncio.CancelledError:
    print("Closing pipe")

async def start_ws_server(args, fd):
  executor = concurrent.futures.ThreadPoolExecutor()
  lock = asyncio.Lock()
  last_gen_code = [None]
  acc = Accumulator()
  connected_clients = set()
  ## Start the websocket server
  websocket_server = await websockets.serve(
    lambda ws, path="": ws_main(ws, path, args, acc, connected_clients), 
    args.ip, args.ws_port)
  print(f"WebSocket server started at ws://{args.ip}:{args.ws_port}")

  broadcast_task = asyncio.create_task(
    broadcast_transcripts_from_pipe(
      args, fd, acc, websocket_server, connected_clients, last_gen_code, lock, executor))

  await asyncio.gather(websocket_server.wait_closed(), broadcast_task)
  executor.shutdown(wait=False)

def start_completion_callback(args, fd):
  try:
    asyncio.run(start_ws_server(args, fd))
  except Exception as e:
    print("Websocket error: " + str(e))
    shutdown(None, None)

def shutdown(sig, frame):
  global ros_available, robot_available, robot_interface, server_thread, asyncio_loop, httpd, ws_server, pipe_descriptor
  print(" Shutting down server.")
  if robot_available and ros_available and robot_interface is not None:
    robot_interface._cancel_goals()
    print("Waiting for 2s to preempt robot actions...")
    time.sleep(2)
    robot_interface.destroy_node()
  if ros_available and rclpy_instance is not None:
    rclpy_instance.shutdown()
  if httpd is not None:
    httpd.server_close()
    httpd.shutdown()
  if server_thread is not None and threading.current_thread() != server_thread:
    server_thread.join()
  running_loop = asyncio.get_running_loop()
  print(" Cancelling asyncio tasks")
  for task in asyncio.all_tasks(loop=running_loop):
    task.cancel()
  print(" Requested stopping asyncio loop")
  running_loop.stop()
  print(" Attempting to close pipe_descriptor")
  if pipe_descriptor is not None:
    ## Need to write something to unblock the read
    os.write(pipe_descriptor, b"@")
    os.close(pipe_descriptor)
    print(" Shutdown closed the pipe_descriptor")
  if ws_server is not None:
    ws_server.close()
  if sig == signal.SIGINT or sig == signal.SIGTERM:
    exit_code = 0
  else:
    exit_code = 1
  sys.exit(exit_code)

def main():
  global server_thread
  global prompt_prefix
  global prompt_suffix
  global ros_available
  global robot_available
  global robot_interface
  global code_timeout
  global model
  global pipe_descriptor
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument('--ip', type=str, help='IP address', default="localhost")
  parser.add_argument('--port', type=int, help='HTML server port number', default=8080)
  parser.add_argument('--ws-port', type=int, help='Websocket server port number', default=8190)
  parser.add_argument("--model-type", choices=["openai", "openai-chat", "palm", "automodel", "hf-textgen"], default="openai-chat")
  parser.add_argument('--model-name', type=str, help='Model name', default='gpt-4')
  parser.add_argument('--tgi-server-url', type=str, help='Text Generation Inference Client URL', default='http://127.0.0.1:8082')
  parser.add_argument('--chat-prompt-prefix', type=Path, help='Prompt prefix for GPT chat completion only', default='code_generation/openai_chat_completion_prefix.py')
  parser.add_argument('--prompt-prefix', type=Path, help='Prompt prefix for all but GPT chat completion', default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=Path, help='Prompt suffix for all but GPT chat completion', default='code_generation/prompt_suffix.py')
  parser.add_argument('--interface-page', type=Path, help='Interface page', default='code_generation/interface.html')
  parser.add_argument('--max-workers', type=int, help='Maximum number of workers', default=1)
  parser.add_argument("--max-tokens", type=int, default=512)
  parser.add_argument("--top-p", type=float, default=0.95)
  parser.add_argument("--temperature", type=float, default=0.2)
  parser.add_argument('--robot', action='store_true', help='Flag to indicate if the robot is available')
  parser.add_argument('--timeout', type=int, help='Code generation timeout in seconds', default=20)
  parser.add_argument('--transcription-pipe', type=Path, help='Pipe from which to read audio transcriptions', default='/tmp/audio_pipe')

  args = parser.parse_args()

  robot_available = args.robot
  code_timeout = args.timeout

  signal.signal(signal.SIGINT, shutdown)

  if robot_available and ros_available:
    from robot_interface.src.robot_client_interface import RobotInterface
    robot_interface = RobotInterface()

  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  model = load_model(args)
  server_thread = threading.Thread(target=serve_interface_html,
                                   name="HTTP server thread",
                                   args=[args])
  server_thread.start()

  ## Opens in read-write mode to prevent EOF.
  pipe_descriptor = os.open(args.transcription_pipe, os.O_RDWR)
  start_completion_callback(args, pipe_descriptor)

if __name__ == "__main__":
  main()
