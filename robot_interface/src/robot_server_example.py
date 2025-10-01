#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time
import sys
import signal
import random
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

rooms = ["kitchen", "Alice's office", "Bob's office", "supply room"]
robot_location = random.choice(rooms)


class RobotActions(Node):
    def __init__(self):
        super().__init__('robot_actions')
        
        # Action servers
        self.go_to_server = ActionServer(
            self, GoTo, "/go_to_server", self.go_to_callback
        )
        self.get_current_location_server = ActionServer(
            self, GetCurrentLocation, "/get_current_location_server", self.get_current_location_callback
        )
        self.is_in_room_server = ActionServer(
            self, IsInRoom, "/is_in_room_server", self.is_in_room_callback
        )
        self.say_server = ActionServer(
            self, Say, "/say_server", self.say_callback
        )
        self.get_all_rooms_server = ActionServer(
            self, GetAllRooms, "/get_all_rooms_server", self.get_all_rooms_callback
        )
        self.ask_server = ActionServer(
            self, Ask, "/ask_server", self.ask_callback
        )
        self.pick_server = ActionServer(
            self, Pick, "/pick_server", self.pick_callback
        )
        self.place_server = ActionServer(
            self, Place, "/place_server", self.place_callback
        )
        print("======= Started all robot action servers =======")

    def go_to_callback(self, goal_handle: ServerGoalHandle):
        global robot_location
        robot_location = goal_handle.request.location

        # Implement code to move the robot to the location
        print(f'go_to("{robot_location}")')

        goal_handle.succeed()
        result = GoTo.Result()
        return result

    def get_current_location_callback(self, goal_handle: ServerGoalHandle):
        global robot_location
        result = GetCurrentLocation.Result()

        # Implement code to get the current location of the robot
        print(f'get_current_location() -> "{robot_location}"')
        result.result = robot_location

        goal_handle.succeed()
        return result

    def is_in_room_callback(self, goal_handle: ServerGoalHandle):
        global robot_location
        object_name = goal_handle.request.object
        result = IsInRoom.Result()

        # Implement code to check if the object is in the room
        result_value = random.choice([True, False])
        print(f'is_in_room("{object_name}") -> {result_value}')
        # Placeholder: return a random result
        result.result = result_value

        goal_handle.succeed()
        return result

    def say_callback(self, goal_handle: ServerGoalHandle):
        message = goal_handle.request.message

        # Implement code to make the robot say the message
        print(f'say("{message}")')

        goal_handle.succeed()
        result = Say.Result()
        return result

    def get_all_rooms_callback(self, goal_handle: ServerGoalHandle):
        global rooms
        result = GetAllRooms.Result()

        # Implement code to get all the rooms
        print(f"get_all_rooms() -> {rooms}")
        result.result = rooms
        goal_handle.succeed()
        return result

    def ask_callback(self, goal_handle: ServerGoalHandle):
        person = goal_handle.request.person
        question = goal_handle.request.question
        options = goal_handle.request.options
        result = Ask.Result()

        # Implement code to ask the question to the person with options and get the response.
        # Placeholder: return a random option from the options list.
        result_value = random.choice(options)
        print(f'ask("{person}", "{question}", {options}) -> {result_value}')
        result.result = result_value

        goal_handle.succeed()
        return result

    def pick_callback(self, goal_handle: ServerGoalHandle):
        obj = goal_handle.request.obj

        # Implement code to pick the object
        print(f'pick("{obj}")')

        goal_handle.succeed()
        result = Pick.Result()
        return result

    def place_callback(self, goal_handle: ServerGoalHandle):
        obj = goal_handle.request.obj

        # Implement code to place the object
        print(f'place("{obj}")')

        goal_handle.succeed()
        result = Place.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    ra = RobotActions()
    try:
        rclpy.spin(ra)
    except KeyboardInterrupt:
        pass
    finally:
        ra.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()