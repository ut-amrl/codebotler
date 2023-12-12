import rclpy
from typing import List
from rclpy.action import ActionClient
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


class RobotInterface(Node):

    def __init__(self):
        super().__init__('robot_action_client')
        self.go_to_client = ActionClient(self, GoTo, '/go_to_server')
        self.get_current_location_client = ActionClient(self, GetCurrentLocation, '/get_current_location_server')
        self.is_in_room_client = ActionClient(self, IsInRoom, '/is_in_room_server')
        self.say_client = ActionClient(self, Say, '/say_server')
        self.get_all_rooms_client = ActionClient(self, GetAllRooms, '/get_all_rooms_server')
        self.ask_client = ActionClient(self, Ask, '/ask_server')
        self.pick_client = ActionClient(self, Pick, '/pick_server')
        self.place_client = ActionClient(self, Place, '/place_server')
        
        
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
        
        self.return_data = None
        self._goal_handle = None

    def _handle_client(self, client, goal, action_name):
        print("Sending goal to " + action_name)
        _send_goal_future = client.send_goal_async(goal)
        _send_goal_future.add_done_callback(self.goal_response_callback)

        while True:
            rclpy.spin_once(self)
            if self.return_data:
                return self.return_data

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError('Goal rejected')

        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.return_data = result

    def go_to(self, location: str):
        self.return_data = None
        goal_msg = GoTo.Goal()
        goal_msg.location = location
        self._handle_client(self.go_to_client, goal_msg, "goto")
    
    def get_current_location(self):
        self.return_data = None
        goal_msg = GetCurrentLocation.Goal()
        result = self._handle_client(self.get_current_location_client, goal_msg, "get_current_location")
        return result.result

    def is_in_room(self, object: str):
        self.return_data = None
        goal_msg = IsInRoom.Goal()
        goal_msg.object = object
        result = self._handle_client(self.is_in_room_client, goal_msg, "goto")
        return result.result
    
    def say(self, message: str):
        self.return_data = None
        goal_msg = Say.Goal()
        goal_msg.message = message
        self._handle_client(self.say_client, goal_msg, "say")

    def get_all_rooms(self):
        self.return_data = None
        goal_msg = GetAllRooms.Goal()
        result = self._handle_client(self.get_all_rooms_client, goal_msg, "get_all_rooms")
        return result.result

    def ask(self, person: str, question: str, options: List[str]):
        self.return_data = None
        goal_msg = Ask.Goal()
        goal_msg.person = person
        goal_msg.question = question
        goal_msg.options = options
        result = self._handle_client(self.ask_client, goal_msg, "ask")
        return result.result

    def pick(self, object: str):
        self.return_data = None 
        goal_msg = Pick.Goal()
        goal_msg.obj = object
        self._handle_client(self.pick_client, goal_msg, "pick")

    def place(self, object: str):
        self.return_data = None
        goal_msg = Place.Goal()
        goal_msg.obj = object
        self._handle_client(self.place_client, goal_msg, "place")

    def _cancel_goals(self):
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

def main(args=None):
    # === testing ===
    rclpy.init(args=args)

    robot = RobotInterface()

    result = robot.goto("kitchen")
    print("result", result)
    result = robot.say("Hello, I am Codebotler!")
    print("result", result)
    result = robot.pick("mug")
    print("result", result)
    result = robot.place("mug")
    print("result", result)
    result = robot.get_all_rooms()
    print("result", result)
    result = robot.get_current_location()
    print("result", result)
    result = robot.ask("Arjun", "How are you?", ["Good", "Bad"])
    print("result", result)
    result = robot.is_in_room("10")
    print("result", result)
    print("==== successful ====")


def execute_task_program(program: str, robot: RobotInterface):
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
            "get_current_location": robot.get_current_location
        }
        program_with_call = program + "\n\ntask_program()\n"
        print("Executing program...")
        exec(program_with_call, namespace)
        print("Program executed successfully.")
    except Exception as e:
        print(
            "There is a problem with executing the program: {}. \nQuitting Execution!! ".format(
                e
            )
        )

if __name__ == '__main__':
    main()