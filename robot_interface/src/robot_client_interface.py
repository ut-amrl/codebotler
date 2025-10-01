#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from typing import List
import time
import signal
import sys
from cobot_codebotler_actions.action import (
    GoTo,
    GetCurrentLocation,
    IsInRoom,
    Say,
    GetAllRooms,
    Ask,
    Pick,
    Place,
)


class RobotExecutionInterrupted(Exception):
    pass


class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface')
        
        # Action clients
        self.go_to_client = ActionClient(self, GoTo, "/go_to_server")
        self.get_current_location_client = ActionClient(self, GetCurrentLocation, "/get_current_location_server")
        self.is_in_room_client = ActionClient(self, IsInRoom, "/is_in_room_server")
        self.say_client = ActionClient(self, Say, "/say_server")
        self.get_all_rooms_client = ActionClient(self, GetAllRooms, "/get_all_rooms_server")
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
        goal_handle = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle)
        
        goal_handle = goal_handle.result()
        if not goal_handle.accepted:
            raise Exception(f"{action_name}() goal was rejected!")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            raise RobotExecutionInterrupted(f"{action_name}()")
        elif result.status != GoalStatus.STATUS_SUCCEEDED:
            raise Exception(f"{action_name}() failed with status {result.status}")
        
        return result.result

    # Get the current location of the robot.
    def get_current_location(self) -> str:
        goal = GetCurrentLocation.Goal()
        return self._handle_client(
            self.get_current_location_client, goal, "get_current_location"
        ).result

    # Get a list of all rooms in the house.
    def get_all_rooms(self) -> List[str]:
        goal = GetAllRooms.Goal()
        return self._handle_client(
            self.get_all_rooms_client, goal, "get_all_rooms"
        ).result

    # Check if an object is in the current room.
    def is_in_room(self, obj: str) -> bool:
        goal = IsInRoom.Goal(object=obj)
        return self._handle_client(self.is_in_room_client, goal, "is_in_room").result

    # Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
    # office"), go_to("Jill's study").
    def go_to(self, location: str) -> None:
        goal = GoTo.Goal(location=location)
        self._handle_client(self.go_to_client, goal, "go_to")

    # Ask a person a question, and offer a set of specific options for the person to
    # respond. Return with the response selected by the person.
    def ask(self, person: str, question: str, options: List[str]) -> str:
        goal = Ask.Goal(person=person, question=question, options=options)
        return self._handle_client(self.ask_client, goal, "ask").result

    # Say the message out loud. Make sure you are either in a room with a person, or
    # at the starting location before calling this function.
    def say(self, message: str) -> None:
        goal = Say.Goal(message=message)
        self._handle_client(self.say_client, goal, "say")

    # Pick up an object from the current room. Make sure you are in the same room as
    # the object before calling this function and not currently holding an object.
    def pick(self, obj: str) -> None:
        goal = Pick.Goal(obj=obj)
        self._handle_client(self.pick_client, goal, "pick")

    # Place an object in the current room. Make sure you are in the same room as the
    # object before calling this function and are currently holding an object.
    def place(self, obj: str) -> None:
        goal = Place.Goal(obj=obj)
        self._handle_client(self.place_client, goal, "place")

    def _cancel_goals(self):
        # Cancel all pending goals
        self.go_to_client.cancel_all_goals()
        self.get_current_location_client.cancel_all_goals()
        self.is_in_room_client.cancel_all_goals()
        self.say_client.cancel_all_goals()
        self.get_all_rooms_client.cancel_all_goals()
        self.ask_client.cancel_all_goals()
        self.pick_client.cancel_all_goals()
        self.place_client.cancel_all_goals()


def execute_task_program(program: str, robot: RobotInterface):
    # every time this is called, a RobotInterface Instance is created in the local scope:
    # might affect performance, but this is clean
    try:
        namespace = {
            "robot": robot,
            "say": robot.say,
            "go_to": robot.go_to,
            "ask": robot.ask,
            "is_in_room": robot.is_in_room,
            "pick": robot.pick,
            "place": robot.place,
            "get_all_rooms": robot.get_all_rooms,
            "get_current_location": robot.get_current_location,
            "time": time,
        }
        program_with_call = program + "\n\ntask_program()\n"
        print("Executing program...")
        exec(program_with_call, namespace)
        print("Program executed successfully.")
    except RobotExecutionInterrupted as i:
        print(
            f"Robot Execution stopped as {i} was interrupted! Terminating execution!!"
        )
    except Exception as e:
        print(
            "There is a problem with executing the program: {}. \nQuitting Execution!! ".format(
                e
            )
        )