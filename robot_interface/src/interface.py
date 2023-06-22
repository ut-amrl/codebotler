#!/usr/bin/env python3
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../code_generation"))

from utilities import *
add_pythonpath_load_amrl_msgs_cd_rel(".", ".")

import rospy
from typing import List
from utilities import process_command_string
from std_msgs.msg import String
import cv2
import numpy as np
import time
import torch
import os
import re
from PIL import Image
import shutil
import signal
import actionlib
from robot_actions_pkg.msg import GoToAction, GoToGoal, GetCurrentLocationAction, GetCurrentLocationGoal, IsInRoomAction, IsInRoomGoal, SayAction, SayGoal, GetAllRoomsAction, GetAllRoomsGoal, AskAction, AskGoal


class RobotInterface:
    def __init__(self):
        self.last_say_bool = None
        self.available_dsl_fns = ["go_to", "get_current_location", "is_in_room", "say", "get_all_rooms", "ask"]
        self.current_code_string = None

        # Action clients
        self.go_to_client = actionlib.SimpleActionClient("/go_to_server", GoToAction)
        self.get_current_location_client = actionlib.SimpleActionClient("/get_current_location_server", GetCurrentLocationAction)
        self.is_in_room_client = actionlib.SimpleActionClient("/is_in_room_server", IsInRoomAction)
        self.say_client = actionlib.SimpleActionClient("/say_server", SayAction)
        self.get_all_rooms_client = actionlib.SimpleActionClient("/get_all_rooms_server", GetAllRoomsAction)
        self.ask_client = actionlib.SimpleActionClient("/ask_server", AskAction)
        self.go_to_client.wait_for_server()
        self.get_current_location_client.wait_for_server()
        self.is_in_room_client.wait_for_server()
        self.say_client.wait_for_server()
        self.get_all_rooms_client.wait_for_server()
        self.ask_client.wait_for_server()

        # Subscribers
        rospy.Subscriber('/chat_commands', String, self.python_cmds_callback, queue_size=10)

    def python_cmds_callback(self, msg):
        self.current_code_string, self.last_say_bool = process_command_string(msg.data, self.available_dsl_fns)
        self.execute()

    def go_to(self, location) -> None:
        g = GoToGoal()
        g.location = location
        self.go_to_client.send_goal(g)
        self.go_to_client.wait_for_result()
        if self.go_to_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Go to preempted!")
        else:
            assert self.go_to_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Go to {location} failed!"

    def get_current_location(self) -> str:
        g = GetCurrentLocationGoal()
        self.get_current_location_client.send_goal(g)
        self.get_current_location_client.wait_for_result()
        if self.get_current_location_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Get current location preempted!")
        else:
            assert self.get_current_location_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Get current location failed!"
        return self.get_current_location_client.get_result().result

    def is_in_room(self, object) -> bool:
        g = IsInRoomGoal()
        g.object = object
        self.is_in_room_client.send_goal(g)
        self.is_in_room_client.wait_for_result()
        if self.is_in_room_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Is in room preempted!")
        else:
            assert self.is_in_room_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Is in room {object} failed!"
        return self.is_in_room_client.get_result().result

    def say(self, message) -> None:
        g = SayGoal()
        g.message = message
        self.say_client.send_goal(g)
        self.say_client.wait_for_result()
        if self.say_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Say preempted!")
        else:
            assert self.say_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Say '{message}' failed!"

    def get_all_rooms(self) -> List[str]:
        g = GetAllRoomsGoal()
        self.get_all_rooms_client.send_goal(g)
        self.get_all_rooms_client.wait_for_result()
        if self.get_all_rooms_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Get all rooms preempted!")
        else:
            assert self.get_all_rooms_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Get all rooms failed!"
        return self.get_all_rooms_client.get_result().result

    def ask(self, person, question, options=None) -> str:
        g = AskGoal()
        g.person = person
        g.question = question
        g.options = options
        self.ask_client.send_goal(g)
        self.ask_client.wait_for_result()
        if self.ask_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Ask preempted!")
        else:
            assert self.ask_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Ask {person} '{question}' with options '{options}' failed!"
        return self.ask_client.get_result().result

    def execute(self):
        exec(self.current_code_string)
        if not self.last_say_bool:
            self.say("Task complete!")


if __name__ == "__main__":
    rospy.init_node('ros_interface', anonymous=False)
    ra = RobotInterface()

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Sending cancel requests to action servers...")
        ra.go_to_client.cancel_all_goals()
        ra.get_current_location_client.cancel_all_goals()
        ra.is_in_room_client.cancel_all_goals()
        ra.say_client.cancel_all_goals()
        ra.get_all_rooms_client.cancel_all_goals()
        ra.ask_client.cancel_all_goals()
        rospy.sleep(5)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
