from benchmark.roboeval_dsl import SPECIAL_DELIM
from typing import List
import re
import numpy as np


class State:
    """
    location name has to be exact
    label, agent name, answer are regex to be matched
    """

    def __init__(
        self,
    ):
        self.locations = []
        self.objects = {}
        self.agents = {}
        self.robot_location = ""

    def addLocation(self, location: str) -> "State":
        self.locations.append(location)
        return self

    def addObject(self, label: str, location: str) -> "State":
        if location not in self.locations:
            raise Exception(
                f"SimulatorError: in addObject: Location {location} not in {self.locations}"
            )
        data = self.objects.get(location, [])
        data.append(label)
        self.objects[location] = data
        return self

    def removeObject(self, label: str, location: str) -> "State":
        if not self.objects.get(location):
            raise Exception(
                f"SimulatorError: in removeObject: Location {location} not in object locations: {list(self.objects.keys())}"
            )

        objs = self.objects[location]
        find_obj = False
        for obj in objs:
            if self.match_value(obj, label):
                objs.remove(obj)
                find_obj = True
                break
        if not find_obj:
            raise Exception(
                f"SimulatorError: in removeObject: Object {label} not in room {location}"
            )

        self.objects[location] = objs
        return self

    def addAgent(self, name: str, location: str, answers: List[str]) -> "State":
        if location not in self.locations:
            raise Exception(
                f"SimulatorError: in addAgent: Location {location} not in {self.locations}"
            )
        data = self.agents.get(location, [])
        data.append((name, answers))
        self.agents[location] = data
        return self

    def addRobotLocation(self, robot_location: str) -> "State":
        if robot_location not in self.locations:
            raise Exception(
                f"SimulatorError: in addRobotLocation: Location {robot_location} not in {self.locations}"
            )
        self.robot_location = robot_location
        return self

    def object_in_room(self, other_obj: str) -> bool:
        curr_loc = self.robot_location
        if not self.objects.get(curr_loc):
            return False
        objs = self.objects[curr_loc]
        for obj in objs:
            if self.match_value(obj, other_obj):
                return True
        return False

    def agent_in_room(self, other_agent: str) -> bool:
        curr_loc = self.robot_location
        if not self.agents.get(curr_loc):
            return False
        agents = self.agents[curr_loc]
        for agent in agents:
            agent_name, agent_answers = agent
            if self.match_value(agent_name, other_agent):
                return True
        return False

    def get_robot_location(self) -> str:
        return self.robot_location

    def update_robot_location(self, location: str) -> None:
        for loc in self.locations:
            if self.match_value(loc, location):
                self.robot_location = loc
                return
        raise Exception(
            f"SimulatorError: in update_robot_location: Location {location} not in {self.locations}"
        )

    def get_all_locations(self) -> List[str]:
        return self.locations

    def ask_agent(self, person, question, options) -> str:
        curr_loc = self.robot_location

        if not self.agents.get(curr_loc):
            raise Exception(
                f"SimulatorError: in Ask: Person {person} does not exist in room {self.get_robot_location()}"
            )

        agents = self.agents[curr_loc]
        for agent in agents:
            agent_name, agent_answers = agent
            if self.match_value(agent_name, person):
                if len(agent_answers) > 1:
                    ans = agent_answers.pop(0)
                else:
                    ans = agent_answers[0]
                for option in options:
                    if self.match_value(ans, option):
                        return option
                if self.match_value(ans, question):
                    for option in options:
                        if self.match_value("yes", option):
                            return option
                else:
                    for option in options:
                        if self.match_value("no", option):
                            return option

        raise Exception(
            f"SimulatorError: in Ask: Person {person} answer {ans} with options {options} \
                    and question {question} in room {self.get_robot_location()}"
        )

    def match_value(self, regex, value) -> bool:
        return bool(re.search(regex, str(value), re.IGNORECASE))

    def __str__(self) -> str:
        return (
            "{\n"
            + '"locations": '
            + str(self.locations)
            + ", \n"
            + '"objects": '
            + str(self.objects)
            + ", \n"
            + '"agents": '
            + str(self.agents)
            + ", \n"
            + '"robot_location": "'
            + str(self.robot_location)
            + '"\n}'
        )


class Robot:
    def __init__(self, state: State):
        self.state = state
        self.is_holding = None

    # Get the current location of the robot.
    def get_current_location(self) -> str:
        loc = self.state.get_robot_location()
        return loc

    # Get a list of all rooms in the house.
    def get_all_rooms(self) -> List[str]:
        rooms = self.state.get_all_locations()
        return rooms

    # Check if an entity (person or object) is in the current room.
    def is_in_room(self, entity: str) -> bool:
        if type(entity) is not str:
            raise TypeError("RobotCheckEntityError: entity must be a string")
        print(
            f"CheckEntity {SPECIAL_DELIM} {entity} {SPECIAL_DELIM} {None}", flush=True
        )
        try:
            is_in_room = self.state.object_in_room(entity) or self.state.agent_in_room(
                entity
            )
        except:
            raise Exception(
                f"RobotCheckEntityError: object_in_room or agent_in_room failed"
            )

        return is_in_room

    # Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
    # office"), go_to("Jill's study").
    def go_to(self, location: str) -> None:
        if type(location) is not str:
            raise TypeError("RobotGoToError: location must be a string")
        print(f"GoTo {SPECIAL_DELIM} {location} {SPECIAL_DELIM} {None}", flush=True)
        try:
            self.state.update_robot_location(location)
        except:
            raise Exception(f"RobotGoToError: update_robot_location failed")

    # Ask a person a question, and offer a set of specific options for the person to
    # respond. Return with the response selected by the person.
    def ask(self, person: str, question: str, options: List[str]) -> str:
        if type(person) is not str:
            raise TypeError("RobotAskError: person must be a string")
        if type(question) is not str:
            raise TypeError("RobotAskError: question must be a string")
        if type(options) is not list:
            raise TypeError("RobotAskError: options must be a list")
        print(f"Ask {SPECIAL_DELIM} {question} {SPECIAL_DELIM} {options}", flush=True)
        if len(options) < 1:
            raise Exception(f"RobotAskError: invalid options")

        try:
            answer = self.state.ask_agent(person, question, options)
        except:
            raise Exception(f"RobotAskError: ask_agent failed")
        return answer

    # Say the message out loud. Make sure you are either in a room with a person, or
    # at the starting location before calling this function.
    def say(self, message: str) -> None:
        if type(message) is not str:
            raise TypeError("RobotSayError: message must be a string")
        print(f"Say {SPECIAL_DELIM} {message} {SPECIAL_DELIM} {None}", flush=True)

    def pick(self, obj: str) -> None:
        if type(obj) is not str:
            raise TypeError("RobotPickError: obj must be a string")
        if self.is_holding is None:
            try:
                self.state.removeObject(obj, self.get_current_location())
                self.is_holding = obj
                print(f"Pick {SPECIAL_DELIM} {obj} {SPECIAL_DELIM} {None}", flush=True)
            except:
                raise Exception(f"RobotPickError: removeObject failed")
        else:
            raise Exception(
                f"RobotPickError: Robot is already holding {self.is_holding}"
            )

    def place(self, obj: str) -> None:
        if type(obj) is not str:
            raise TypeError("RobotPlaceError: obj must be a string")
        if self.is_holding == obj:
            try:
                self.is_holding = None
                self.state.addObject(obj, self.get_current_location())
                print(f"Place {SPECIAL_DELIM} {obj} {SPECIAL_DELIM} {None}", flush=True)
            except:
                raise Exception(f"RobotPlaceError: addObject failed")
        else:
            raise Exception(f"RobotPlaceError: Robot is not holding {obj}")


class RobotExecution:
    def __init__(self, state: State):
        self.state = state
        self.is_holding = None

    # Get the current location of the robot.
    def get_current_location(self) -> str:
        loc = self.state.get_robot_location()
        return loc

    # Get a list of all rooms in the house.
    def get_all_rooms(self) -> List[str]:
        rooms = self.state.get_all_locations()
        return rooms

    def is_in_room(self, entity: str) -> bool:
        if type(entity) is not str:
            raise TypeError("RobotCheckEntityError: entity must be a string")
        # randomize check
        choices = [True, False]
        random_choice = np.random.choice(choices)
        return random_choice

    def go_to(self, location: str) -> None:
        if type(location) is not str:
            raise TypeError("RobotGoToError: location must be a string")
        try:
            self.state.update_robot_location(location)
        except:
            raise Exception(f"RobotGoToError: update_robot_location failed")

    def ask(self, person: str, question: str, options: List[str]) -> str:
        if type(person) is not str:
            raise TypeError("RobotAskError: person must be a string")
        if type(question) is not str:
            raise TypeError("RobotAskError: question must be a string")
        if type(options) is not list:
            raise TypeError("RobotAskError: options must be a list")

        if len(options) < 1 or type(options[0]) is not str:
            raise Exception(f"RobotAskError: invalid options")

        # random answering
        random_answer = np.random.choice(options)
        return random_answer

    def say(self, message: str) -> None:
        if type(message) is not str:
            raise TypeError("RobotSayError: message must be a string")

    def pick(self, obj: str) -> None:
        if type(obj) is not str:
            raise TypeError("RobotPickError: obj must be a string")
        if self.is_holding is None:
            self.is_holding = obj
        else:
            raise Exception(
                f"RobotPickError: Robot is already holding {self.is_holding}"
            )

    def place(self, obj: str) -> None:
        if type(obj) is not str:
            raise TypeError("RobotPlaceError: obj must be a string")
        if self.is_holding != obj:
            raise Exception(f"RobotPlaceError: Robot is not holding {obj}")
        self.is_holding = None
