import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from codebotler_actions.action import (
    GoTo,
    GetCurrentLocation,
    IsInRoom,
    Say,
    GetAllRooms,
    Ask,
    Pick,
    Place
)
import random
import time 
class RobotActions(Node):

    def __init__(self):
        super().__init__('robot_action_server')
        self.rooms = ["kitchen", "Alice's office", "Bob's office", "supply room"]
        self.robot_location = random.choice(self.rooms)

        self.go_to_server =ActionServer(self, GoTo, '/go_to_server', self.go_to)
        self.get_current_location_server = ActionServer(self, GetCurrentLocation, '/get_current_location_server', self.get_current_location)
        self.is_in_room_server = ActionServer(self, IsInRoom, '/is_in_room_server', self.is_in_room)
        self.say_server = ActionServer(self, Say, '/say_server', self.say)
        self.get_all_rooms_server = ActionServer(self, GetAllRooms, '/get_all_rooms_server', self.get_all_rooms)
        self.ask_server = ActionServer(self, Ask, '/ask_server', self.ask)
        self.pick_server = ActionServer(self, Pick, '/pick_server', self.pick)
        self.place_server =ActionServer(self, Place, '/place_server', self.place)

        print("======= Started all robot action servers =======")

    def go_to(self, goal_handle):
        self.robot_location = goal_handle.request.location 

        # Implement code to move the robot to the location
        print(f'go_to("{self.robot_location}")')

        result = GoTo.Result()
        goal_handle.succeed()
        return result
    
    def get_current_location(self, goal_handle):
        # Implement code to get the current location of the robot
        print(f'get_current_location() -> "{self.robot_location}"')

        result = GetCurrentLocation.Result() 
        result.result = self.robot_location
        goal_handle.succeed()
        return result

    def is_in_room(self, goal_handle):
        object = goal_handle.request.object

        # Implement code to check if the object is in the room
        result_bool = random.choice([True, False])
        print(f'is_in_room("{object}") -> {result_bool}')

        result = IsInRoom.Result()
        result.result = result_bool 
        goal_handle.succeed()
        return result

    def say(self, goal_handle):
        message = goal_handle.request.message
        time.sleep(1)
        # Implement code to make the robot say the message
        print(f'say("{message}")')

        result = Say.Result()
        goal_handle.succeed()
        return result

    def get_all_rooms(self, goal_handle):
        # Implement code to get all the rooms
        print(f"get_all_rooms() -> {self.rooms}") 

        result = GetAllRooms.Result()
        result.result = self.rooms
        goal_handle.succeed()
        return result

    def ask(self, goal_handle):
        person = goal_handle.request.person
        question = goal_handle.request.question
        options = goal_handle.request.options
        
        # Implement code to ask the question to the person with options and get the response.
        # Placeholder: return a random option from the options list.
        answer = random.choice(options)
        print(f'ask("{person}", "{question}", {options}) -> {answer}')

        result = Ask.Result()
        result.result = answer
        goal_handle.succeed()
        return result

    def pick(self, goal_handle):
        object = goal_handle.request.obj

        # Implement code to make the robot pick the object
        print(f'pick("{object}")')

        result = Pick.Result()
        goal_handle.succeed() 
        return result

    def place(self, goal_handle):
        object = goal_handle.request.obj

        # Implement code to make the robot place the object at the location
        print(f'place("{object}"')

        result = Place.Result()
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    robot_action_server = RobotActions()

    rclpy.spin(robot_action_server)


if __name__ == '__main__':
    main()