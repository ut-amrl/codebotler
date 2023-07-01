#!/usr/bin/env python3

import rospy
import time
import sys
import signal
import actionlib
import random
from robot_actions_pkg.msg import GoToAction, GetCurrentLocationAction, IsInRoomAction, SayAction, GetAllRoomsAction, AskAction
from robot_actions_pkg.msg import GoToResult, GetCurrentLocationResult, IsInRoomResult, SayResult, GetAllRoomsResult, AskResult

rooms = ["kitchen", "Alice's office", "Bob's office", "supply room"]
robot_location = random.choice(rooms)

class RobotActions:
    def __init__(self):
        # Action servers
        self.go_to_server = actionlib.SimpleActionServer("/go_to_server", GoToAction, self.go_to, auto_start=False)
        self.get_current_location_server = actionlib.SimpleActionServer("/get_current_location_server", GetCurrentLocationAction, self.get_current_location, auto_start=False)
        self.is_in_room_server = actionlib.SimpleActionServer("/is_in_room_server", IsInRoomAction, self.is_in_room, auto_start=False)
        self.say_server = actionlib.SimpleActionServer("/say_server", SayAction, self.say, auto_start=False)
        self.get_all_rooms_server = actionlib.SimpleActionServer("/get_all_rooms_server", GetAllRoomsAction, self.get_all_rooms, auto_start=False)
        self.ask_server = actionlib.SimpleActionServer("/ask_server", AskAction, self.ask, auto_start=False)
        self.go_to_server.start()
        self.get_current_location_server.start()
        self.is_in_room_server.start()
        self.say_server.start()
        self.get_all_rooms_server.start()
        self.ask_server.start()

        print("======= Started all robot action servers =======")

    def go_to(self, goal):
        global robot_location
        robot_location = goal.location

        # Implement code to move the robot to the location
        print(f"go_to(\"{robot_location}\")")

        self.go_to_server.set_succeeded()

    def get_current_location(self, goal):
        global robot_location
        r = GetCurrentLocationResult()

        # Implement code to get the current location of the robot
        print(f"get_current_location() -> \"{robot_location}\"")
        r.result = robot_location

        self.get_current_location_server.set_succeeded(r)

    def is_in_room(self, goal):
        global robot_location
        object = goal.object
        r = IsInRoomResult()

        # Implement code to check if the object is in the room
        result = random.choice([True, False])
        print(f"is_in_room(\"{object}\") -> {result}")
        # Placeholder: return a random result
        r.result = result

        self.is_in_room_server.set_succeeded(r)

    def say(self, goal):
        message = goal.message

        # Implement code to make the robot say the message
        print(f"say(\"{message}\")")

        self.say_server.set_succeeded()

    def get_all_rooms(self, goal):
        global rooms
        r = GetAllRoomsResult()

        # Implement code to get all the rooms
        print(f"get_all_rooms() -> {rooms}")
        r.result = rooms
        self.get_all_rooms_server.set_succeeded(r)

    def ask(self, goal):
        person = goal.person
        question = goal.question
        options = goal.options
        r = AskResult()

        # Implement code to ask the question to the person with options and get the response.
        # Placeholder: return a random option from the options list.
        result = random.choice(options)
        print(f"ask(\"{person}\", \"{question}\", {options}) -> {result}")
        r.result = result

        self.ask_server.set_succeeded(r)


if __name__ == "__main__":
    rospy.init_node('robot_low_level_actions', anonymous=False)
    ra = RobotActions()
    rospy.spin()