#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from typing import List
import time
import signal
import sys
from robot_actions_pkg.action import (
    GoTo,
    GetCurrentLocation,
    IsInRoom,
    Say,
    GetAllRooms,
    Ask,
    Pick,
    Place,
    PickUp,
    PutDown,
    PutIntoBasket,
    RetrieveFromBasket,
    GetReachableLocationsAroundObject,
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
        self.pick_up_client = ActionClient(self, PickUp, "/pick_up_server")
        self.put_down_client = ActionClient(self, PutDown, "/put_down_server")
        self.put_into_basket_client = ActionClient(self, PutIntoBasket, "/put_into_basket_server")
        self.retrieve_from_basket_client = ActionClient(self, RetrieveFromBasket, "/retrieve_from_basket_server")
        self.get_reachable_locations_around_object_client = ActionClient(self, GetReachableLocationsAroundObject, "/get_reachable_locations_around_object_server")

        print("====== Waiting for robot action servers... ======")
        self.go_to_client.wait_for_server()
        self.get_current_location_client.wait_for_server()
        self.is_in_room_client.wait_for_server()
        self.say_client.wait_for_server()
        self.get_all_rooms_client.wait_for_server()
        self.ask_client.wait_for_server()
        self.pick_client.wait_for_server()
        self.place_client.wait_for_server()
        self.pick_up_client.wait_for_server()
        self.put_down_client.wait_for_server()
        self.put_into_basket_client.wait_for_server()
        self.retrieve_from_basket_client.wait_for_server()
        self.get_reachable_locations_around_object_client.wait_for_server()
        print("======= Connected to robot action servers =======")

    @staticmethod
    def _handle_client(client, goal, action_name):
        goal_handle = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(None, goal_handle)
        
        goal_handle = goal_handle.result()
        if not goal_handle.accepted:
            raise Exception(f"{action_name}() goal was rejected!")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(None, result_future)
        
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
    # object before calling this function and is currently holding an object.
    def place(self, obj: str) -> None:
        goal = Place.Goal(obj=obj)
        self._handle_client(self.place_client, goal, "place")

    # Pick up an object from the environment. Make sure you can reach the object from your
    # current location first.
    def pick_up(self, obj: str) -> None:
        goal = PickUp.Goal(obj=obj)
        self._handle_client(self.pick_up_client, goal, "pick_up")
    
    # Pick up an object from the environment. Make sure you can reach the object from your
    # current location first.
    def put_down(self, obj: str, dest: str) -> None:
        goal = PutDown.Goal(obj=obj, location=dest)
        self._handle_client(self.put_down_client, goal, "put_down")

    # Place an already held object into the basket
    def put_into_basket(self, obj: str) -> None:
        goal = PutIntoBasket.Goal(obj=obj)
        self._handle_client(self.put_into_basket_client, goal, "put_into_basket")

    # Retrieve an item that is in the basket from the basket, and make sure you are holding it
    def retrieve_from_basket(self, obj: str) -> None:
        goal = RetrieveFromBasket.Goal(obj=obj)
        self._handle_client(self.retrieve_from_basket_client, goal, "retrieve_from_basket")

    # Given an object, returns the list of locations within a certain location around the object
    def get_reachable_locations_around_object(self, obj: str) -> List[str]:
        goal = GetReachableLocationsAroundObject.Goal(obj=obj)
        return self._handle_client(self.get_reachable_locations_around_object_client, goal, "get_reachable_locations_around_object").locations

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
        self.pick_up_client.cancel_all_goals()
        self.put_down_client.cancel_all_goals()
        self.put_into_basket_client.cancel_all_goals()
        self.retrieve_from_basket_client.cancel_all_goals()
        self.get_reachable_locations_around_object_client.cancel_all_goals()


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
            "pick_up": robot.pick_up,
            "put_down": robot.put_down,
            "put_into_basket": robot.put_into_basket,
            "retrieve_from_basket": robot.retrieve_from_basket,
            "get_reachable_locations_around_object": robot.get_reachable_locations_around_object,
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