#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import String
from robot_client.msg import AskAction, GetAllRoomsAction, GetCurrentLocationAction, GoToAction, IsInRoomAction, SayAction
from robot_client.msg import AskGoal, GetAllRoomsGoal, GetCurrentLocationGoal, GoToGoal, IsInRoomGoal, SayGoal
 

class Robot:
    def __init__(self):
        self.ask_client = actionlib.SimpleActionClient('/robot_ask', AskAction)
        self.get_all_rooms_client = actionlib.SimpleActionClient('/robot_get_all_rooms', GetAllRoomsAction)
        self.get_current_location_client = actionlib.SimpleActionClient('/robot_get_current_location', GetCurrentLocationAction)
        self.go_to_client = actionlib.SimpleActionClient('/robot_go_to', GoToAction)
        self.is_in_room_client = actionlib.SimpleActionClient('/robot_is_in_room', IsInRoomAction)
        self.say_client = actionlib.SimpleActionClient('/robot_say', SayAction)

        self.ask_client.wait_for_server()
        self.get_all_rooms_client.wait_for_server()
        self.get_current_location_client.wait_for_server()
        self.go_to_client.wait_for_server()
        self.is_in_room_client.wait_for_server()
        self.say_client.wait_for_server()

        rospy.Subscriber('/generated_code', String, self.python_cmds_callback, queue_size=10)
        print("====== all server connected ========")

    def python_cmds_callback(self, msg):
        try:
            grounding = "say = self.say\n" + \
                "go_to = self.go_to\n" + \
                "ask = self.ask\n" + \
                "is_in_room = self.is_in_room\n" + \
                "get_all_rooms = self.get_all_rooms\n" + \
                "get_current_location = self.get_current_location\n"
            program = msg.data
            p = grounding + program 
            exec(p)
        except Exception as e:
            print("There is a problem with the generated program: {}. \n Quitting Execution!! ".format(e))

    def send_goal_wrapper(self, client, goal):
        client.send_goal(goal)

        client.wait_for_result()

        return client.get_result()

    # Get the current location of the robot.
    def get_current_location(self) -> str :
        goal = GetCurrentLocationGoal()
        return self.send_goal_wrapper(self.get_current_location_client, goal).result

    # Get a list of all rooms in the house.
    def get_all_rooms(self) -> list[str] :
        goal = GetAllRoomsGoal()
        return self.send_goal_wrapper(self.get_all_rooms_client, goal).result


    # Check if an object is in the current room.
    def is_in_room(self, obj : str) -> bool :
        goal = IsInRoomGoal(object=obj)
        return self.send_goal_wrapper(self.is_in_room_client, goal).result

    # Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
    # office"), go_to("Jill's study").
    def go_to(self, location : str) -> None :
        goal = GoToGoal(location=location)
        return self.send_goal_wrapper(self.go_to_client, goal)
        
    # Ask a person a question, and offer a set of specific options for the person to
    # respond. Return with the response selected by the person.
    def ask(self, person : str, question : str, options: list[str]) -> str :
        goal = AskGoal(person=person,question=question,options=options)
        return self.send_goal_wrapper(self.ask_client, goal).result

    # Say the message out loud. Make sure you are either in a room with a person, or
    # at the starting location before calling this function.
    def say(self, message : str) -> None :
        goal = SayGoal(message=message)
        return self.send_goal_wrapper(self.say_client, goal)

if __name__ == "__main__":
    rospy.init_node('robot_client')
    robot = Robot()
    rospy.spin()
