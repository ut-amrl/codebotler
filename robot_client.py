#!/usr/bin/env python3

import json
import os
import selectors
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cobot_codebotler_actions.action import (
    Ask,
    GetAllRooms,
    GetCurrentLocation,
    GoTo,
    IsInRoom,
    Pick,
    Place,
    Say,
)

STATUS_PREFIX = "__CODEBOTLER_ACTION_STATUS__ "
STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
    GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
    GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
}
CANCEL_GRACE_SECONDS = 8.0
CANCEL_SIGNAL = signal.SIGUSR1 if hasattr(signal, "SIGUSR1") else signal.SIGTERM


class TaskCancelled(RuntimeError):
    pass


def current_task_program_line(stack_depth=2):
    try:
        start_line = int(os.getenv("CODEBOTLER_TASK_PROGRAM_START_LINE", "0"))
    except ValueError:
        start_line = 0
    if start_line <= 0:
        return None
    try:
        line_number = sys._getframe(stack_depth).f_lineno
    except ValueError:
        return None
    program_line = line_number - start_line + 1
    return program_line if program_line > 0 else None


class SimulatedRobotInterface:
    def __init__(self, status_callback=None, sleep_seconds=5.0, cancel_event=None):
        self.status_callback = status_callback
        self.sleep_seconds = sleep_seconds
        self.cancel_event = cancel_event or threading.Event()
        self._action_sequence = 0

    def destroy_node(self):
        pass

    def cancel_current_goal(self):
        self.cancel_event.set()

    def _cancel_requested(self):
        return self.cancel_event.is_set()

    def _next_action_sequence(self):
        self._action_sequence += 1
        return self._action_sequence

    def _emit_status(self, call, status, detail="", action_sequence=None, source_line=None):
        if self.status_callback is None:
            return
        payload = {
            "call": call,
            "status": status,
            "detail": detail,
            "timestamp": time.time(),
        }
        if action_sequence is not None:
            payload["action_sequence"] = action_sequence
        if source_line is not None:
            payload["source_line"] = source_line
        self.status_callback(payload)

    def _simulate(self, call, result=None, source_line=None):
        print(f"{call}: simulated DSL call")
        action_sequence = self._next_action_sequence()
        if self._cancel_requested():
            self._emit_status(
                call,
                "STATUS_CANCELED",
                "Task canceled before simulated action.",
                action_sequence,
                source_line,
            )
            raise TaskCancelled("Task canceled.")
        self._emit_status(call, "GOAL_SENT", "Simulated goal sent.", action_sequence, source_line)
        self._emit_status(call, "STATUS_ACCEPTED", "Simulated goal accepted.", action_sequence, source_line)

        deadline = time.monotonic() + self.sleep_seconds
        next_update = time.monotonic()
        while True:
            if self._cancel_requested():
                self._emit_status(call, "STATUS_CANCELED", "Simulated action canceled.", action_sequence, source_line)
                raise TaskCancelled("Task canceled.")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            now = time.monotonic()
            if now >= next_update:
                self._emit_status(call, "STATUS_EXECUTING", "Simulating action.", action_sequence, source_line)
                next_update = now + 2.0
            time.sleep(min(0.1, remaining))

        self._emit_status(call, "STATUS_SUCCEEDED", "Simulated action succeeded.", action_sequence, source_line)
        return result

    def get_current_location(self) -> str:
        return self._simulate("get_current_location()", "labapt", current_task_program_line())

    def get_all_rooms(self) -> List[str]:
        return self._simulate("get_all_rooms()", ["labapt"], current_task_program_line())

    def is_in_room(self, obj: str) -> bool:
        return self._simulate(f"is_in_room({obj!r})", True, current_task_program_line())

    def go_to(self, location: str) -> None:
        self._simulate(f"go_to({location!r})", source_line=current_task_program_line())

    def ask(self, person: str, question: str, options: List[str]) -> str:
        answer = options[0] if options else ""
        return self._simulate(
            f"ask({person!r}, {question!r}, {options!r})",
            answer,
            current_task_program_line(),
        )

    def say(self, message: str) -> None:
        self._simulate(f"say({message!r})", source_line=current_task_program_line())

    def pick(self, obj: str) -> None:
        self._simulate(f"pick({obj!r})", source_line=current_task_program_line())

    def place(self, obj: str) -> None:
        self._simulate(f"place({obj!r})", source_line=current_task_program_line())


class RobotInterface(Node):
    def __init__(self, status_callback=None, cancel_event=None):
        super().__init__("robot_interface")
        self.status_callback = status_callback
        self.cancel_event = cancel_event or threading.Event()
        self._goal_lock = threading.Lock()
        self._active_goal_handle = None
        self._action_sequence = 0

        self.go_to_client = ActionClient(self, GoTo, "/go_to_server")
        self.get_current_location_client = ActionClient(
            self, GetCurrentLocation, "/get_current_location_server"
        )
        self.is_in_room_client = ActionClient(self, IsInRoom, "/is_in_room_server")
        self.say_client = ActionClient(self, Say, "/say_server")
        self.get_all_rooms_client = ActionClient(
            self, GetAllRooms, "/get_all_rooms_server"
        )
        self.ask_client = ActionClient(self, Ask, "/ask_server")
        self.pick_client = ActionClient(self, Pick, "/pick_server")
        self.place_client = ActionClient(self, Place, "/place_server")

        print("====== Waiting for robot action servers... ======")
        self._wait_for_server(self.go_to_client, "go_to")
        self._wait_for_server(self.get_current_location_client, "get_current_location")
        self._wait_for_server(self.is_in_room_client, "is_in_room")
        self._wait_for_server(self.say_client, "say")
        self._wait_for_server(self.get_all_rooms_client, "get_all_rooms")
        self._wait_for_server(self.ask_client, "ask")
        self._wait_for_server(self.pick_client, "pick")
        self._wait_for_server(self.place_client, "place")
        print("======= Connected to robot action servers =======")

    def cancel_current_goal(self):
        self.cancel_event.set()

    def _cancel_requested(self):
        return self.cancel_event.is_set()

    def _next_action_sequence(self):
        self._action_sequence += 1
        return self._action_sequence

    def _raise_if_cancel_requested(self, call, action_sequence=None, source_line=None):
        if self._cancel_requested():
            self._emit_status(call, "STATUS_CANCELED", "Task canceled.", action_sequence, source_line)
            raise TaskCancelled("Task canceled.")

    def _wait_for_server(self, client, action_name):
        while rclpy.ok() and not client.wait_for_server(timeout_sec=0.5):
            if self._cancel_requested():
                raise TaskCancelled(f"Task canceled while waiting for {action_name} server.")

    def _spin_once(self, call, action_sequence=None, source_line=None):
        try:
            rclpy.spin_once(self, timeout_sec=0.1)
        except ExternalShutdownException as exc:
            if self._cancel_requested():
                self._emit_status(call, "STATUS_CANCELED", "Task canceled during ROS spin.", action_sequence, source_line)
                raise TaskCancelled("Task canceled.") from exc
            raise

    def _emit_status(self, call, status, detail="", action_sequence=None, source_line=None):
        if self.status_callback is None:
            return
        payload = {
            "call": call,
            "status": status,
            "detail": detail,
            "timestamp": time.time(),
        }
        if action_sequence is not None:
            payload["action_sequence"] = action_sequence
        if source_line is not None:
            payload["source_line"] = source_line
        self.status_callback(payload)

    def _request_goal_cancel(self, goal_handle, call, action_sequence=None, source_line=None):
        self._emit_status(call, "STATUS_CANCELING", "Sending action cancel request.", action_sequence, source_line)
        try:
            cancel_future = goal_handle.cancel_goal_async()
            deadline = time.monotonic() + 2.0
            while rclpy.ok() and not cancel_future.done() and time.monotonic() < deadline:
                self._spin_once(call, action_sequence, source_line)
        except Exception as exc:
            self._emit_status(call, "STATUS_CANCELING", f"Action cancel request failed: {exc}", action_sequence, source_line)

    def _handle_client(self, client, goal, action_name, call, source_line=None):
        print(f"{action_name}(): sending goal")
        action_sequence = self._next_action_sequence()
        self._raise_if_cancel_requested(call, action_sequence, source_line)
        self._emit_status(call, "GOAL_SENT", "Waiting for server acceptance.", action_sequence, source_line)
        goal_handle_future = client.send_goal_async(goal)
        cancel_during_accept = False
        cancel_accept_deadline = None
        while rclpy.ok() and not goal_handle_future.done():
            if self._cancel_requested():
                if not cancel_during_accept:
                    self._emit_status(call, "STATUS_CANCELING", "Waiting for goal handle before canceling.", action_sequence, source_line)
                    cancel_during_accept = True
                    cancel_accept_deadline = time.monotonic() + 2.0
                elif time.monotonic() >= cancel_accept_deadline:
                    self._emit_status(call, "STATUS_CANCELED", "Task canceled before goal acceptance completed.", action_sequence, source_line)
                    raise TaskCancelled("Task canceled.")
            self._spin_once(call, action_sequence, source_line)

        goal_handle = goal_handle_future.result()
        if cancel_during_accept or self._cancel_requested():
            if goal_handle is not None and goal_handle.accepted:
                self._request_goal_cancel(goal_handle, call, action_sequence, source_line)
            self._emit_status(call, "STATUS_CANCELED", "Task canceled.", action_sequence, source_line)
            raise TaskCancelled("Task canceled.")
        if goal_handle is None:
            self._emit_status(call, "NO_GOAL_RESPONSE", "No goal response received.", action_sequence, source_line)
            raise RuntimeError(f"{action_name}() did not receive a goal response!")
        if not goal_handle.accepted:
            print(f"{action_name}(): goal rejected")
            self._emit_status(call, "GOAL_REJECTED", "Server rejected the goal.", action_sequence, source_line)
            raise RuntimeError(f"{action_name}() goal was rejected!")

        print(f"{action_name}(): goal accepted; waiting for result")
        self._emit_status(call, "STATUS_ACCEPTED", "Goal accepted; waiting for result.", action_sequence, source_line)

        result_future = goal_handle.get_result_async()
        with self._goal_lock:
            self._active_goal_handle = goal_handle
        try:
            next_update = time.monotonic()
            while rclpy.ok() and not result_future.done():
                if self._cancel_requested():
                    self._request_goal_cancel(goal_handle, call, action_sequence, source_line)
                    self._emit_status(call, "STATUS_CANCELED", "Task canceled.", action_sequence, source_line)
                    raise TaskCancelled("Task canceled.")
                self._spin_once(call, action_sequence, source_line)
                now = time.monotonic()
                if now >= next_update:
                    self._emit_status(call, "STATUS_EXECUTING", "Waiting for action result.", action_sequence, source_line)
                    next_update = now + 2.0
        finally:
            with self._goal_lock:
                if self._active_goal_handle is goal_handle:
                    self._active_goal_handle = None

        result = result_future.result()
        if result is None:
            self._emit_status(call, "NO_RESULT", "No action result received.", action_sequence, source_line)
            raise RuntimeError(f"{action_name}() did not receive a result!")
        status_name = STATUS_NAMES.get(result.status, f"STATUS_{result.status}")
        self._emit_status(call, status_name, "Action result received.", action_sequence, source_line)
        if result.status == GoalStatus.STATUS_CANCELED:
            raise TaskCancelled("Task canceled.")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{action_name}() failed with status {result.status}")
        self._raise_if_cancel_requested(call, action_sequence, source_line)

        print(f"{action_name}(): succeeded")
        return result.result

    def get_current_location(self) -> str:
        goal = GetCurrentLocation.Goal()
        return self._handle_client(
            self.get_current_location_client,
            goal,
            "get_current_location",
            "get_current_location()",
            current_task_program_line(),
        ).result

    def get_all_rooms(self) -> List[str]:
        goal = GetAllRooms.Goal()
        return self._handle_client(
            self.get_all_rooms_client,
            goal,
            "get_all_rooms",
            "get_all_rooms()",
            current_task_program_line(),
        ).result

    def is_in_room(self, obj: str) -> bool:
        goal = IsInRoom.Goal(object=obj)
        return self._handle_client(
            self.is_in_room_client,
            goal,
            "is_in_room",
            f"is_in_room({obj!r})",
            current_task_program_line(),
        ).result

    def go_to(self, location: str) -> None:
        goal = GoTo.Goal(location=location)
        self._handle_client(self.go_to_client, goal, "go_to", f"go_to({location!r})", current_task_program_line())

    def ask(self, person: str, question: str, options: List[str]) -> str:
        goal = Ask.Goal(person=person, question=question, options=options)
        return self._handle_client(
            self.ask_client,
            goal,
            "ask",
            f"ask({person!r}, {question!r}, {options!r})",
            current_task_program_line(),
        ).result

    def say(self, message: str) -> None:
        goal = Say.Goal(message=message)
        self._handle_client(self.say_client, goal, "say", f"say({message!r})", current_task_program_line())

    def pick(self, obj: str) -> None:
        goal = Pick.Goal(obj=obj)
        self._handle_client(self.pick_client, goal, "pick", f"pick({obj!r})", current_task_program_line())

    def place(self, obj: str) -> None:
        goal = Place.Goal(obj=obj)
        self._handle_client(self.place_client, goal, "place", f"place({obj!r})", current_task_program_line())

def execute_task_program(program: str, use_robot: bool, status_callback=None, cancel_event=None):
    """
    Execute the generated task program in a subprocess.

    The subprocess creates its own real or simulated interface so ROS state and
    user-code failures stay isolated from the web server.
    """
    cancel_event = cancel_event or threading.Event()
    codebotler_root = Path(__file__).resolve().parent
    indented_program = "\n        ".join(program.splitlines())

    script_content = f"""
import json
import os
import signal
import sys
import threading
import rclpy
import traceback

sys.path.insert(0, {str(codebotler_root)!r})

from robot_client import RobotInterface, SimulatedRobotInterface, TaskCancelled

STATUS_PREFIX = {STATUS_PREFIX!r}
USE_ROBOT = {use_robot!r}
CANCEL_SIGNAL = {int(CANCEL_SIGNAL)!r}
TASK_PROGRAM_START_LINE = 0
cancel_event = threading.Event()
cancel_announced = False
node = None


def write_action_status(payload):
    print(STATUS_PREFIX + json.dumps(payload), flush=True)


def request_cancel(signum=None, frame=None):
    global cancel_announced
    cancel_event.set()
    if not cancel_announced:
        write_action_status({{"call": "task_program()", "status": "STATUS_CANCELING", "detail": "Task cancellation requested."}})
        cancel_announced = True
    if node is not None:
        node.cancel_current_goal()


def main():
    global node
    if USE_ROBOT:
        rclpy.init()
    signal.signal(CANCEL_SIGNAL, request_cancel)
    try:
        write_action_status({{"call": "task_program()", "status": "STATUS_EXECUTING", "detail": "Program started."}})
        if cancel_event.is_set():
            raise TaskCancelled("Task canceled.")
        if USE_ROBOT:
            node = RobotInterface(status_callback=write_action_status, cancel_event=cancel_event)
        else:
            node = SimulatedRobotInterface(status_callback=write_action_status, cancel_event=cancel_event)

        say = node.say
        go_to = node.go_to
        ask = node.ask
        is_in_room = node.is_in_room
        pick = node.pick
        place = node.place
        get_all_rooms = node.get_all_rooms
        get_current_location = node.get_current_location

        print("Executing program in subprocess...")
        if cancel_event.is_set():
            raise TaskCancelled("Task canceled.")
        os.environ["CODEBOTLER_TASK_PROGRAM_START_LINE"] = str(TASK_PROGRAM_START_LINE)

        # __CODEBOTLER_TASK_PROGRAM_START__
        {indented_program}

        if "task_program" in locals():
            task_program()
        else:
            print("Error: task_program() not defined by generated code.")

        if cancel_event.is_set():
            raise TaskCancelled("Task canceled.")
        print("Program executed successfully.")
        write_action_status({{"call": "None", "status": "IDLE", "detail": "Program executed successfully."}})

    except TaskCancelled as exc:
        print(f"Task canceled: {{exc}}")
        write_action_status({{"call": "None", "status": "STATUS_CANCELED", "detail": str(exc)}})
        sys.exit(130)
    except Exception as exc:
        traceback.print_exc()
        print(f"Error executing generated program: {{exc}}")
        write_action_status({{"call": "None", "status": "STATUS_ABORTED", "detail": str(exc)}})
        sys.exit(1)
    finally:
        if node:
            node.destroy_node()
        if USE_ROBOT:
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
"""
    marker = "        # __CODEBOTLER_TASK_PROGRAM_START__"
    script_lines = script_content.splitlines()
    try:
        program_start_line = script_lines.index(marker) + 2
    except ValueError:
        program_start_line = 0
    script_content = script_content.replace(
        "TASK_PROGRAM_START_LINE = 0",
        f"TASK_PROGRAM_START_LINE = {program_start_line}",
        1,
    )

    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as temp_file:
        temp_file_path = temp_file.name
        temp_file.write(script_content)
        temp_file.flush()

    try:
        env = os.environ.copy()
        print(f"Starting {temp_file_path}")
        process = subprocess.Popen(
            [sys.executable, temp_file_path],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )

        def handle_output_line(line):
            line = line.rstrip("\n")
            if line.startswith(STATUS_PREFIX):
                payload_text = line[len(STATUS_PREFIX):]
                try:
                    payload = json.loads(payload_text)
                    if status_callback is not None:
                        status_callback(payload)
                except json.JSONDecodeError:
                    print(line)
            else:
                print(line)

        cancel_sent_at = None

        def signal_subprocess(sig, process_group=True):
            if not process_group:
                try:
                    process.send_signal(sig)
                except ProcessLookupError:
                    pass
                except Exception:
                    pass
                return
            try:
                os.killpg(os.getpgid(process.pid), sig)
            except ProcessLookupError:
                pass
            except Exception:
                try:
                    process.send_signal(sig)
                except Exception:
                    pass

        def request_subprocess_cancel():
            nonlocal cancel_sent_at
            if cancel_sent_at is not None or process.poll() is not None:
                return
            cancel_sent_at = time.monotonic()
            if status_callback is not None:
                status_callback({
                    "call": "task_program()",
                    "status": "STATUS_CANCELING",
                    "detail": "Stopping task process.",
                    "timestamp": time.time(),
                })
            signal_subprocess(CANCEL_SIGNAL, process_group=False)

        selector = selectors.DefaultSelector()
        if process.stdout is not None:
            selector.register(process.stdout, selectors.EVENT_READ)

        while process.poll() is None:
            if cancel_event.is_set():
                request_subprocess_cancel()
            if (
                cancel_sent_at is not None
                and time.monotonic() - cancel_sent_at > CANCEL_GRACE_SECONDS
                and process.poll() is None
            ):
                print("Task process did not stop after cancel request; killing it.")
                signal_subprocess(signal.SIGKILL)
                break

            for key, _ in selector.select(timeout=0.1):
                line = key.fileobj.readline()
                if line:
                    handle_output_line(line)
                else:
                    try:
                        selector.unregister(key.fileobj)
                    except Exception:
                        pass

        if process.stdout is not None:
            for line in process.stdout:
                handle_output_line(line)

        returncode = process.wait()
        if cancel_event.is_set():
            if status_callback is not None:
                status_callback({
                    "call": "None",
                    "status": "STATUS_CANCELED",
                    "detail": "Task canceled.",
                    "timestamp": time.time(),
                })
        elif returncode != 0:
            raise subprocess.CalledProcessError(returncode, process.args)
    except subprocess.CalledProcessError as exc:
        print(f"Subprocess execution failed with return code {exc.returncode}")
        if status_callback is not None:
            status_callback({
                "call": "None",
                "status": "STATUS_ABORTED",
                "detail": f"Task process failed with return code {exc.returncode}.",
                "timestamp": time.time(),
            })
    except Exception as exc:
        print(f"Error launching subprocess: {exc}")
        if status_callback is not None:
            status_callback({
                "call": "None",
                "status": "STATUS_ABORTED",
                "detail": f"Error launching task process: {exc}",
                "timestamp": time.time(),
            })
    finally:
        print("Task complete")
        try:
            os.unlink(temp_file_path)
        except OSError:
            pass
