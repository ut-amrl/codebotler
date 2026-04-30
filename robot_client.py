#!/usr/bin/env python3

import os
import subprocess
import sys
import tempfile
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


class RobotExecutionInterrupted(Exception):
    pass


class RobotInterface(Node):
    def __init__(self):
        super().__init__("robot_interface")

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

    def _handle_client(self, client, goal, action_name):
        goal_handle_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle_future)

        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"{action_name}() goal was rejected!")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            raise RobotExecutionInterrupted(f"{action_name}()")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{action_name}() failed with status {result.status}")

        return result.result

    def get_current_location(self) -> str:
        goal = GetCurrentLocation.Goal()
        return self._handle_client(
            self.get_current_location_client, goal, "get_current_location"
        ).result

    def get_all_rooms(self) -> List[str]:
        goal = GetAllRooms.Goal()
        return self._handle_client(self.get_all_rooms_client, goal, "get_all_rooms").result

    def is_in_room(self, obj: str) -> bool:
        goal = IsInRoom.Goal(object=obj)
        return self._handle_client(self.is_in_room_client, goal, "is_in_room").result

    def go_to(self, location: str) -> None:
        goal = GoTo.Goal(location=location)
        self._handle_client(self.go_to_client, goal, "go_to")

    def ask(self, person: str, question: str, options: List[str]) -> str:
        goal = Ask.Goal(person=person, question=question, options=options)
        return self._handle_client(self.ask_client, goal, "ask").result

    def say(self, message: str) -> None:
        goal = Say.Goal(message=message)
        self._handle_client(self.say_client, goal, "say")

    def pick(self, obj: str) -> None:
        goal = Pick.Goal(obj=obj)
        self._handle_client(self.pick_client, goal, "pick")

    def place(self, obj: str) -> None:
        goal = Place.Goal(obj=obj)
        self._handle_client(self.place_client, goal, "place")

    def _cancel_goals(self):
        pass


def execute_task_program(program: str, robot: RobotInterface):
    """
    Execute the generated task program in a subprocess.

    The robot argument is intentionally ignored. The subprocess creates its own
    RobotInterface so ROS state and user-code failures stay isolated.
    """
    codebotler_root = Path(__file__).resolve().parent
    indented_program = "\n        ".join(program.splitlines())

    script_content = f"""
import sys
import rclpy
import traceback

sys.path.insert(0, {str(codebotler_root)!r})

from robot_client import RobotInterface, RobotExecutionInterrupted


def main():
    rclpy.init()
    node = None
    try:
        node = RobotInterface()

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

    except RobotExecutionInterrupted as interrupted:
        print(f"Robot execution stopped because {{interrupted}} was interrupted.")
    except Exception as exc:
        traceback.print_exc()
        print(f"Error executing generated program: {{exc}}")
        sys.exit(1)
    finally:
        if node:
            node.destroy_node()
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
        )
        returncode = process.wait()
        if returncode != 0:
            raise subprocess.CalledProcessError(returncode, process.args)
    except subprocess.CalledProcessError as exc:
        print(f"Subprocess execution failed with return code {exc.returncode}")
    except Exception as exc:
        print(f"Error launching subprocess: {exc}")
    finally:
        print("Task complete")
