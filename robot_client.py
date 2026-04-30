#!/usr/bin/env python3

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
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


class SimulatedRobotInterface:
    def __init__(self, status_callback=None, sleep_seconds=5.0):
        self.status_callback = status_callback
        self.sleep_seconds = sleep_seconds

    def destroy_node(self):
        pass

    def _emit_status(self, call, status, detail=""):
        if self.status_callback is None:
            return
        self.status_callback({
            "call": call,
            "status": status,
            "detail": detail,
            "timestamp": time.time(),
        })

    def _simulate(self, call, result=None):
        print(f"{call}: simulated DSL call")
        self._emit_status(call, "GOAL_SENT", "Simulated goal sent.")
        self._emit_status(call, "STATUS_ACCEPTED", "Simulated goal accepted.")

        deadline = time.monotonic() + self.sleep_seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._emit_status(call, "STATUS_EXECUTING", "Simulating action.")
            time.sleep(min(2.0, remaining))

        self._emit_status(call, "STATUS_SUCCEEDED", "Simulated action succeeded.")
        return result

    def get_current_location(self) -> str:
        return self._simulate("get_current_location()", "labapt")

    def get_all_rooms(self) -> List[str]:
        return self._simulate("get_all_rooms()", ["labapt"])

    def is_in_room(self, obj: str) -> bool:
        return self._simulate(f"is_in_room({obj!r})", True)

    def go_to(self, location: str) -> None:
        self._simulate(f"go_to({location!r})")

    def ask(self, person: str, question: str, options: List[str]) -> str:
        answer = options[0] if options else ""
        return self._simulate(f"ask({person!r}, {question!r}, {options!r})", answer)

    def say(self, message: str) -> None:
        self._simulate(f"say({message!r})")

    def pick(self, obj: str) -> None:
        self._simulate(f"pick({obj!r})")

    def place(self, obj: str) -> None:
        self._simulate(f"place({obj!r})")


class RobotInterface(Node):
    def __init__(self, status_callback=None):
        super().__init__("robot_interface")
        self.status_callback = status_callback

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
        self.go_to_client.wait_for_server()
        self.get_current_location_client.wait_for_server()
        self.is_in_room_client.wait_for_server()
        self.say_client.wait_for_server()
        self.get_all_rooms_client.wait_for_server()
        self.ask_client.wait_for_server()
        self.pick_client.wait_for_server()
        self.place_client.wait_for_server()
        print("======= Connected to robot action servers =======")

    def _emit_status(self, call, status, detail=""):
        if self.status_callback is None:
            return
        self.status_callback({
            "call": call,
            "status": status,
            "detail": detail,
            "timestamp": time.time(),
        })

    def _handle_client(self, client, goal, action_name, call):
        print(f"{action_name}(): sending goal")
        self._emit_status(call, "GOAL_SENT", "Waiting for server acceptance.")
        goal_handle_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle_future)

        goal_handle = goal_handle_future.result()
        if goal_handle is None:
            self._emit_status(call, "NO_GOAL_RESPONSE", "No goal response received.")
            raise RuntimeError(f"{action_name}() did not receive a goal response!")
        if not goal_handle.accepted:
            print(f"{action_name}(): goal rejected")
            self._emit_status(call, "GOAL_REJECTED", "Server rejected the goal.")
            raise RuntimeError(f"{action_name}() goal was rejected!")

        print(f"{action_name}(): goal accepted; waiting for result")
        self._emit_status(call, "STATUS_ACCEPTED", "Goal accepted; waiting for result.")

        result_future = goal_handle.get_result_async()
        next_update = time.monotonic()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_update:
                self._emit_status(call, "STATUS_EXECUTING", "Waiting for action result.")
                next_update = now + 2.0

        result = result_future.result()
        if result is None:
            self._emit_status(call, "NO_RESULT", "No action result received.")
            raise RuntimeError(f"{action_name}() did not receive a result!")
        status_name = STATUS_NAMES.get(result.status, f"STATUS_{result.status}")
        self._emit_status(call, status_name, "Action result received.")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{action_name}() failed with status {result.status}")

        print(f"{action_name}(): succeeded")
        return result.result

    def get_current_location(self) -> str:
        goal = GetCurrentLocation.Goal()
        return self._handle_client(
            self.get_current_location_client,
            goal,
            "get_current_location",
            "get_current_location()",
        ).result

    def get_all_rooms(self) -> List[str]:
        goal = GetAllRooms.Goal()
        return self._handle_client(
            self.get_all_rooms_client,
            goal,
            "get_all_rooms",
            "get_all_rooms()",
        ).result

    def is_in_room(self, obj: str) -> bool:
        goal = IsInRoom.Goal(object=obj)
        return self._handle_client(
            self.is_in_room_client,
            goal,
            "is_in_room",
            f"is_in_room({obj!r})",
        ).result

    def go_to(self, location: str) -> None:
        goal = GoTo.Goal(location=location)
        self._handle_client(self.go_to_client, goal, "go_to", f"go_to({location!r})")

    def ask(self, person: str, question: str, options: List[str]) -> str:
        goal = Ask.Goal(person=person, question=question, options=options)
        return self._handle_client(
            self.ask_client,
            goal,
            "ask",
            f"ask({person!r}, {question!r}, {options!r})",
        ).result

    def say(self, message: str) -> None:
        goal = Say.Goal(message=message)
        self._handle_client(self.say_client, goal, "say", f"say({message!r})")

    def pick(self, obj: str) -> None:
        goal = Pick.Goal(obj=obj)
        self._handle_client(self.pick_client, goal, "pick", f"pick({obj!r})")

    def place(self, obj: str) -> None:
        goal = Place.Goal(obj=obj)
        self._handle_client(self.place_client, goal, "place", f"place({obj!r})")

def execute_task_program(program: str, use_robot: bool, status_callback=None):
    """
    Execute the generated task program in a subprocess.

    The subprocess creates its own real or simulated interface so ROS state and
    user-code failures stay isolated from the web server.
    """
    codebotler_root = Path(__file__).resolve().parent
    indented_program = "\n        ".join(program.splitlines())

    script_content = f"""
import json
import sys
import rclpy
import traceback

sys.path.insert(0, {str(codebotler_root)!r})

from robot_client import RobotInterface, SimulatedRobotInterface

STATUS_PREFIX = {STATUS_PREFIX!r}
USE_ROBOT = {use_robot!r}


def write_action_status(payload):
    print(STATUS_PREFIX + json.dumps(payload), flush=True)


def main():
    if USE_ROBOT:
        rclpy.init()
    node = None
    try:
        write_action_status({{"call": "task_program()", "status": "STATUS_EXECUTING", "detail": "Program started."}})
        if USE_ROBOT:
            node = RobotInterface(status_callback=write_action_status)
        else:
            node = SimulatedRobotInterface(status_callback=write_action_status)

        say = node.say
        go_to = node.go_to
        ask = node.ask
        is_in_room = node.is_in_room
        pick = node.pick
        place = node.place
        get_all_rooms = node.get_all_rooms
        get_current_location = node.get_current_location

        print("Executing program in subprocess...")

        {indented_program}

        if "task_program" in locals():
            task_program()
        else:
            print("Error: task_program() not defined by generated code.")

        print("Program executed successfully.")
        write_action_status({{"call": "None", "status": "IDLE", "detail": "Program executed successfully."}})

    except Exception as exc:
        traceback.print_exc()
        print(f"Error executing generated program: {{exc}}")
        write_action_status({{"call": "None", "status": "STATUS_ABORTED", "detail": str(exc)}})
        sys.exit(1)
    finally:
        if node:
            node.destroy_node()
        if USE_ROBOT:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
"""

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
        )
        for line in process.stdout:
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
        returncode = process.wait()
        if returncode != 0:
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
