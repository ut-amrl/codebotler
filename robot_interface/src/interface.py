#!/usr/bin/env python3

import rospy
from typing import List
import time
import signal
import actionlib
import sys
from robot_actions_pkg.msg import ExecuteAction, ExecuteFeedback, ExecuteResult, GoToAction, GoToGoal, GetCurrentLocationAction, GetCurrentLocationGoal, IsInRoomAction, IsInRoomGoal, SayAction, SayGoal, GetAllRoomsAction, GetAllRoomsGoal, AskAction, AskGoal


class RobotExecutionInterrupted(Exception):
    pass


class RobotInterface:
    def __init__(self):
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

        # Action servers
        self.execute_server = actionlib.SimpleActionServer("/execute_server", ExecuteAction, self.execute, auto_start=False)
        self.execute_server.start()

        print("====== all clients and servers connected ========")

    # Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
    # office"), go_to("Jill's study").
    def go_to(self, location) -> None:
        g = GoToGoal()
        g.location = location
        self.go_to_client.send_goal(g)
        self.go_to_client.wait_for_result()
        if self.go_to_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            raise RobotExecutionInterrupted("go_to()")
        else:
            assert self.go_to_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Go to {location} failed!"

    # Get the current location of the robot.
    def get_current_location(self) -> str:
        g = GetCurrentLocationGoal()
        self.get_current_location_client.send_goal(g)
        self.get_current_location_client.wait_for_result()
        if self.get_current_location_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            raise RobotExecutionInterrupted("get_current_location()")
        else:
            assert self.get_current_location_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Get current location failed!"
        return self.get_current_location_client.get_result().result

    # Check if an object is in the current room.
    def is_in_room(self, object) -> bool:
        g = IsInRoomGoal()
        g.object = object
        self.is_in_room_client.send_goal(g)
        self.is_in_room_client.wait_for_result()
        if self.is_in_room_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            raise RobotExecutionInterrupted("is_in_room()")
        else:
            assert self.is_in_room_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Is in room {object} failed!"
        return self.is_in_room_client.get_result().result

    # Say the message out loud. Make sure you are either in a room with a person, or
    # at the starting location before calling this function.
    def say(self, message) -> None:
        g = SayGoal()
        g.message = message
        self.say_client.send_goal(g)
        self.say_client.wait_for_result()
        if self.say_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            raise RobotExecutionInterrupted("say()")
        else:
            assert self.say_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Say '{message}' failed!"

    # Get a list of all rooms in the house.
    def get_all_rooms(self) -> List[str]:
        g = GetAllRoomsGoal()
        self.get_all_rooms_client.send_goal(g)
        self.get_all_rooms_client.wait_for_result()
        if self.get_all_rooms_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            raise RobotExecutionInterrupted("get_all_rooms()")
        else:
            assert self.get_all_rooms_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Get all rooms failed!"
        return self.get_all_rooms_client.get_result().result

    # Ask a person a question, and offer a set of specific options for the person to
    # respond. Return with the response selected by the person.
    def ask(self, person, question, options=None) -> str:
        g = AskGoal()
        g.person = person
        g.question = question
        g.options = options
        self.ask_client.send_goal(g)
        self.ask_client.wait_for_result()
        if self.ask_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            raise RobotExecutionInterrupted("ask()")
        else:
            assert self.ask_client.get_state() == actionlib.GoalStatus.SUCCEEDED, f"Ask {person} '{question}' with options '{options}' failed!"
        return self.ask_client.get_result().result

    def execute(self, goal):
        program = goal.program
        try:
            grounding = "say = self.say\n" + \
                "go_to = self.go_to\n" + \
                "ask = self.ask\n" + \
                "is_in_room = self.is_in_room\n" + \
                "get_all_rooms = self.get_all_rooms\n" + \
                "get_current_location = self.get_current_location\n"
            p = grounding + program
            exec(p)
            task_program()
            self.execute_server.set_succeeded()
        except RobotExecutionInterrupted as i:
            print(f"Robot Execution stopped as {i} was interrupted! Terminating execution!!")
            self.execute_server.set_preempted()
        except Exception as e:
            print("There is a problem with the executing the program: {}. \n Quitting Execution!! ".format(e))
            self.execute_server.set_preempted()


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
