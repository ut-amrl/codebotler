import typing

class Object:
  def __init__(self, label : str, location : str):
    self.label = label
    self.location = location

  def __str__(self) -> str:
    return "{ label: \"" + self.label + "\", location: \"" + \
        self.location + "\" }"

  def __repr__(self) -> str:
    return "{ label: \"" + self.label + "\", location: \"" + \
        self.location + "\" }"

class InteractiveAgent:
  def __init__(self, name : str, location : str, answers : list[str]) :
    self.name = name
    self.location = location
    self.answers = answers

  def __str__(self) -> str:
    return "{ name: \"" + self.name + \
        "\", location: \"" + self.location + "\", answers: " + \
        str(self.answers) + " }"

  def __repr__(self) -> str:
    return "{ name: \"" + self.name + \
        "\", location: \"" + self.location + "\", answers: " + \
        str(self.answers) + " }"

class State:
  def __init__(self,
               locations : list[str],
               objects : list[Object],
               interactive_agents : list[InteractiveAgent],
               robot_location : str):
    self.locations = locations
    self.objects = objects
    self.interactive_agents = interactive_agents
    self.robot_location = robot_location

  def __str__(self) -> str:
    return "{\n" + \
        "locations: " + str(self.locations) + ", \n" + \
        "objects: " + str(self.objects) + ", \n" + \
        "interactive_agents: " + str(self.interactive_agents) + ", \n" + \
        "robot_location: " + str(self.robot_location) + "\n}"

class TraceElement:
  # def __init__(self, function_name : str, args : list[str | list[str]]) -> None:
  def __init__(self, function_name : str,
               args : list[typing.Union[str, list[str]]]) -> None:
    self.function_name = function_name
    self.args = args

  def __str__(self) -> str:
    return "{ function_name: " + self.function_name + ", args: " + \
        str(self.args) + " }"

  def __repr__(self) -> str:
    return "{ function_name: " + self.function_name + ", args: " + \
        str(self.args) + " }"

state = State(
    locations=["Arjun's office",
               "Kitchen",
               "Joydeep's office",
               "Conference room"],
  objects = [
    Object(label = "pringles", location = "Kitchen" ),
    Object(label = "black backpack", location = "Conference room"),
    Object(label = "person", location = "Arjun's office"),
    Object(label = "person", location = "Joydeep's office")
    ],
  interactive_agents = [
    InteractiveAgent(name = "Arjun",
                     location = "Arjun's office",
                     answers = ["yes", "5", "Gummies"]),
    InteractiveAgent(name = "Arjun",
                     location = "Arjun's office",
                     answers = ["no", "1"]),
    InteractiveAgent(name = "Joydeep",
                     location = "Conference room",
                     answers = ["no", "Licorice"])
  ],
  robot_location = "Arjun's office"
)

trace : list[TraceElement] = []

# Get the current location of the robot.
def get_current_location() -> str :
  trace.append(TraceElement("get_current_location", []))
  return state.robot_location

# Get a list of all rooms in the house.
def get_all_rooms() -> list[str] :
  trace.append(TraceElement("get_all_rooms", []))
  return state.locations

# Check if an object is in the current room.
def is_in_room(object : str) -> bool :
  trace.append(TraceElement("is_in_room", [object]))
  for o in state.objects:
    if o.location == state.robot_location and o.label == object:
      return True
  return False

# Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
# office"), go_to("Jill's study").
def go_to(location : str) -> None :
  global state
  trace.append(TraceElement("go_to", [location]))
  assert location in state.locations
  state.robot_location = location
  # print(state)

# Ask a person a question, and offer a set of specific options for the person to
# respond. Return with the response selected by the person.
def ask(person : str, question : str, options: list[str]) -> str :
  trace.append(TraceElement("ask", [person, question, options]))
  for p in state.interactive_agents:
    # if p.location == state.robot_location and p.name == person:
    if p.location == state.robot_location:
      # print(f"matched location {state.robot_location}")
      for a in p.answers:
        # print(f"option: {a}")
        for o in options:
          if o == a:
            return a
      # print(f"no match between {options} and {p.answers}")
      # No matching answer found.
      return options[0]
      assert False
  # No matching person found at the location.
  return options[0]
  assert False

# Say the message out loud. Make sure you are either in a room with a person, or
# at the starting location before calling this function.
def say(message : str) -> None :
  trace.append(TraceElement("say", [message]))
  pass

def run_program(program : str, state : State) -> list[TraceElement] :
  exec(program)
  return trace

program = "my_rooms = get_all_rooms()\n" + \
          "for r in my_rooms:\n" + \
          "  go_to(r)\n" + \
          "  say(f\"I am in {r}\")\n"

program = "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\ncandies = [\"Chocolate\", \"Gummies\", \"Licorice\"]\ncandies_count = {candy : 0 for candy in candies}\nfor room in list_of_rooms:\n    if \"office\" not in room:\n        continue\n    go_to(room)\n    if is_in_room(\"person\"):\n        response = ask(\"Person\", \"Which kind of candy would you like?\", candies)\n        candies_count[response] += 1\ngo_to(start_loc)\nfor candy, count in candies_count.items():\n    say(\"We need to buy \" + str(count) + \" \" + candy)"
# program = "say(\"Hello, world!\")"
# program = "r = 10\n"
print("=======================\nProgram:\n=======================")
print(program)
print("=======================\nState:\n=======================")
print(state)
print("=======================\nTrace:\n=======================")
t = run_program(program, state)
print(*t, sep="\n")