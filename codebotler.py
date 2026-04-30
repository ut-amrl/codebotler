#! /usr/bin/env python3

import ast
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
import uuid
from pathlib import Path

import rclpy
from openai import OpenAI

TASK_PROGRAM_HEADER = "def task_program():"
REPO_ROOT = Path(__file__).resolve().parent
INTERFACE_FILE = REPO_ROOT / "interface.html"
PROMPT_FILE = REPO_ROOT / "openai_chat_completion_prefix.py"
CONSOLE_LOG_DIR = Path(os.getenv("CODEBOTLER_CONSOLE_LOG_DIR", "/tmp/codebotler_console_logs"))
CONSOLE_LOGS = {
  "codebotler": CONSOLE_LOG_DIR / "codebotler.log",
  "actions": CONSOLE_LOG_DIR / "actions.log",
  "gui": CONSOLE_LOG_DIR / "gui.log",
}


def enable_console_log(name):
  CONSOLE_LOG_DIR.mkdir(parents=True, exist_ok=True)
  log_path = CONSOLE_LOGS[name]
  log_path.write_text("")

  original_stdout_fd = os.dup(1)
  read_fd, write_fd = os.pipe()
  os.dup2(write_fd, 1)
  os.dup2(write_fd, 2)
  os.close(write_fd)

  try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
  except Exception:
    pass

  def pump_console():
    with open(log_path, "ab", buffering=0) as log:
      while True:
        try:
          data = os.read(read_fd, 4096)
        except OSError:
          break
        if not data:
          break
        log.write(data)
        try:
          os.write(original_stdout_fd, data)
        except OSError:
          pass

  threading.Thread(target=pump_console, name=f"{name}_console_log", daemon=True).start()


enable_console_log("codebotler")

rclpy.init()

httpd = None
server_thread = None
ws_server = None
execution_manager = None


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


def make_event_callback(loop, clients):
  def event_callback(payload):
    try:
      asyncio.run_coroutine_threadsafe(broadcast_json(clients, payload), loop)
    except RuntimeError:
      pass
  return event_callback


def read_last_lines(path, max_lines=100):
  try:
    with open(path, "r", errors="replace") as f:
      lines = f.read().splitlines()
  except OSError:
    return []
  return lines[-max_lines:]


async def stream_console_logs(clients):
  while True:
    logs = {
      name: read_last_lines(path, 100)
      for name, path in CONSOLE_LOGS.items()
    }
    await broadcast_json(clients, {
      "type": "console_logs",
      "logs": logs,
      "timestamp": time.time(),
    })
    await asyncio.sleep(2.0)


class TaskExecution:
  def __init__(self, execution_id, owner, code, use_robot, status_callback, on_finish):
    self.execution_id = execution_id
    self.owner = owner
    self.code = code
    self.use_robot = use_robot
    self.status_callback = status_callback
    self.on_finish = on_finish
    self.cancel_event = threading.Event()
    self.finished_event = threading.Event()
    self.cancel_requested = False
    self.thread = threading.Thread(
      target=self._run,
      name=f"robot_execute_{execution_id[:8]}",
    )

  def start(self):
    self.thread.start()

  def is_alive(self):
    return self.thread.is_alive() and not self.finished_event.is_set()

  def cancel(self, reason):
    if self.finished_event.is_set():
      return False
    if not self.cancel_requested:
      self.cancel_requested = True
      if self.status_callback is not None:
        self.status_callback({
          "call": "task_program()",
          "status": "STATUS_CANCELING",
          "detail": reason,
          "timestamp": time.time(),
        })
    self.cancel_event.set()
    return True

  def _run(self):
    try:
      from robot_client import execute_task_program
      execute_task_program(
        self.code,
        self.use_robot,
        status_callback=self.status_callback,
        cancel_event=self.cancel_event,
      )
    finally:
      self.finished_event.set()
      self.on_finish(self)


class ExecutionManager:
  def __init__(self, status_callback, event_callback):
    self.status_callback = status_callback
    self.event_callback = event_callback
    self.active_execution = None
    self.lock = threading.Lock()

  def start(self, owner, code, use_robot):
    if code is None or not code.strip():
      print("No generated code available. Ignoring execute request.")
      if self.status_callback is not None:
        self.status_callback({
          "call": "None",
          "status": "IDLE",
          "detail": "No generated code available.",
          "timestamp": time.time(),
        })
      return None, "No generated code available."

    with self.lock:
      if self.active_execution is not None and self.active_execution.is_alive():
        return None, "A task program is already running."

      execution_id = uuid.uuid4().hex
      execution = TaskExecution(
        execution_id=execution_id,
        owner=owner,
        code=code,
        use_robot=use_robot,
        status_callback=self.status_callback,
        on_finish=self.finish,
      )
      self.active_execution = execution

    print(f"Received execution request {execution_id} for program:\n```python\n{code}\n```")
    execution.start()
    return execution, None

  def finish(self, execution):
    with self.lock:
      if self.active_execution is execution:
        self.active_execution = None
    if self.event_callback is not None:
      self.event_callback({
        "type": "execution_finished",
        "execution_id": execution.execution_id,
      })

  def snapshot(self):
    with self.lock:
      execution = self.active_execution
      if execution is None or not execution.is_alive():
        return None
      return {
        "type": "execution_started",
        "execution_id": execution.execution_id,
        "program": execution.code,
      }

  def cancel(self, execution_id=None, owner=None, reason="Cancel requested."):
    with self.lock:
      execution = self.active_execution
      if execution is None or not execution.is_alive():
        return False
      if execution_id is not None and execution.execution_id != execution_id:
        return False
      if owner is not None and execution.owner is not owner:
        return False

    return execution.cancel(reason)

  def cancel_for_owner(self, owner, reason):
    return self.cancel(owner=owner, reason=reason)


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


def model_supports_stop_sequences(model: str) -> bool:
  return not model.startswith("gpt-5")


def extract_fenced_blocks(text: str) -> list[str]:
  blocks = []
  current = []
  in_fence = False

  for line in text.splitlines():
    if line.strip().startswith("```"):
      if in_fence:
        blocks.append("\n".join(current).strip())
        current = []
        in_fence = False
      else:
        current = []
        in_fence = True
      continue
    if in_fence:
      current.append(line)

  return [block for block in blocks if block]


def first_task_program_block(code: str) -> str:
  lines = code.strip().splitlines()
  start = None
  for i, line in enumerate(lines):
    if line.lstrip().startswith("def task_program("):
      start = i
      break

  if start is None:
    return code.strip()

  base_indent = len(lines[start]) - len(lines[start].lstrip())
  block = [lines[start]]
  for line in lines[start + 1:]:
    stripped = line.strip()
    indent = len(line) - len(line.lstrip())
    if stripped and indent <= base_indent and not line.lstrip().startswith("#"):
      break
    block.append(line)

  return "\n".join(block).strip()


def wrap_body_in_task_program(code: str) -> str:
  body = code.strip()
  if not body:
    raise ValueError("OpenAI returned empty code.")
  indented_body = "\n".join(
    f"    {line}" if line.strip() else ""
    for line in body.splitlines()
  )
  return f"{TASK_PROGRAM_HEADER}\n{indented_body}".strip()


def ensure_task_program(code: str) -> str:
  code = code.strip()
  fenced_blocks = extract_fenced_blocks(code)
  if fenced_blocks:
    code = next(
      (block for block in fenced_blocks if "def task_program(" in block),
      fenced_blocks[0],
    )
  code = first_task_program_block(code)
  if not code.lstrip().startswith("def task_program("):
    code = wrap_body_in_task_program(code)

  try:
    tree = ast.parse(code)
  except SyntaxError:
    code = first_task_program_block(code)
    tree = ast.parse(code)

  for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name == "task_program":
      source = ast.get_source_segment(code, node)
      if not source:
        break
      parsed_code = source.strip()
      if "```" in parsed_code:
        raise ValueError("Parsed task program still contains Markdown fences.")
      ast.parse(parsed_code)
      return parsed_code

  raise ValueError("Generated code does not define task_program().")


class OpenAICodeGenerator:
  def __init__(self, model: str):
    print("Using OpenAI Chat model:", model)
    self.model = model
    self.client = OpenAI(api_key=load_openai_api_key())
    self.messages = load_prompt_messages(PROMPT_FILE)

  def generate_one(self, prompt, stop_sequences, temperature, top_p, max_tokens):
    if len(stop_sequences) > 4:
      raise ValueError("OpenAI API only supports up to 4 stop sequences.")

    request = {
      "model": self.model,
      "messages": self.messages + [{"role": "user", "content": prompt}],
      "temperature": temperature,
      "top_p": top_p,
      "max_completion_tokens": max_tokens,
    }
    if stop_sequences and model_supports_stop_sequences(self.model):
      request["stop"] = stop_sequences

    response = self.client.chat.completions.create(**request)
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

async def handle_message(websocket, message, args, generator, use_robot, manager):
  data = json.loads(message)
  if data['type'] == 'code':
    print("Received code generation request")
    code, time_str = generate_code(data['prompt'], args, generator)
    response = {"code": f"{code}", "timing": time_str}
    await websocket.send(json.dumps(response))
  elif data['type'] == 'execute':
    print("Executing generated code...")
    code = data.get('code', '')
    execution, error = manager.start(websocket, code, use_robot)
    if execution is None:
      await websocket.send(json.dumps({
        "type": "execution_rejected",
        "detail": error,
      }))
    else:
      manager.event_callback({
        "type": "execution_started",
        "execution_id": execution.execution_id,
        "program": code,
      })
  elif data['type'] == 'cancel':
    execution_id = data.get("execution_id")
    canceled = manager.cancel(
      execution_id=execution_id,
      owner=None if execution_id else websocket,
      reason=data.get("reason", "Cancel requested from UI."),
    )
    await websocket.send(json.dumps({
      "type": "cancel_ack",
      "canceled": canceled,
    }))
  else:
    print("Unknown message type: " + data['type'])

async def ws_main(websocket, _path, args, generator, use_robot, connected_clients, manager):
  ## Whenever a client connects to the websocket, ws_main is called,
  ## and websocket refers to the connection with that specific client.
  connected_clients.add(websocket)
  print(f"Client has connected; {len(connected_clients)} clients...")
  snapshot = manager.snapshot()
  if snapshot is not None:
    await websocket.send(json.dumps(snapshot))

  async def receive_messages():
    try:
      async for message in websocket:
        await handle_message(websocket, message, args, generator, use_robot, manager)
    except websockets.exceptions.ConnectionClosedOK:
      print("Client has disconnected...")
    except websockets.exceptions.ConnectionClosedError as e:
      print(f"Client has disconnected with error: {e}")
    finally:
      manager.cancel_for_owner(websocket, "Client disconnected; canceling active task.")
      connected_clients.discard(websocket)
      print(f"A client has been removed; {len(connected_clients)} clients...")

  await receive_messages()

async def start_ws_server(args, generator, use_robot):
  global ws_server, execution_manager
  connected_clients = set()
  loop = asyncio.get_running_loop()
  status_callback = make_status_callback(loop, connected_clients)
  event_callback = make_event_callback(loop, connected_clients)
  execution_manager = ExecutionManager(status_callback, event_callback)
  asyncio.create_task(stream_console_logs(connected_clients))
  ## Start the websocket server
  ws_server = await websockets.serve(
    lambda ws, path="": ws_main(
      ws, path, args, generator, use_robot, connected_clients, execution_manager),
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
  global server_thread, httpd, ws_server, execution_manager
  print(" Shutting down server.")
  if execution_manager is not None:
    execution_manager.cancel(reason="CodeBotler server is shutting down.")
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
