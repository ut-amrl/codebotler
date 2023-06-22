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
asp_trace : list[str] = []
trace_t = 0

# Get the current location of the robot.
def get_current_location() -> str :
  global trace_t
  asp_trace.append(f"t_get_current_location({trace_t}).")
  trace_t += 1
  trace.append(TraceElement("get_current_location", []))
  return state.robot_location

# Get a list of all rooms in the house.
def get_all_rooms() -> list[str] :
  global trace_t
  asp_trace.append(f"t_get_all_rooms({trace_t}).")
  trace_t += 1
  trace.append(TraceElement("get_all_rooms", []))
  return state.locations

# Check if an object is in the current room.
def is_in_room(object : str) -> bool :
  global trace_t
  asp_trace.append(f"t_is_in_room(\"{object}\",{trace_t}).")
  trace_t += 1
  trace.append(TraceElement("is_in_room", [object]))
  for o in state.objects:
    if o.location == state.robot_location and o.label == object:
      return True
  return False

# Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
# office"), go_to("Jill's study").
def go_to(location : str) -> None :
  global trace_t
  asp_trace.append(f"t_go_to(\"{location}\",{trace_t}).")
  trace_t += 1
  global state
  trace.append(TraceElement("go_to", [location]))
  assert location in state.locations
  state.robot_location = location
  # print(state)

# Ask a person a question, and offer a set of specific options for the person to
# respond. Return with the response selected by the person.
def ask(person : str, question : str, options: list[str]) -> str :
  global trace_t
  options_str = ""
  for o in options:
    options_str += f"[{o}],"
  asp_trace.append(f"t_ask(\"{person}\", \"{question}\", \"{options_str}\", {trace_t}).")
  trace_t += 1
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
  global trace_t
  asp_trace.append(f"t_say(\"{message}\",{trace_t}).")
  trace_t += 1
  trace.append(TraceElement("say", [message]))
  pass

def run_program(program : str, state : State) -> list[TraceElement] :
  exec(program)
  return trace

def state_to_asp(state: State) -> str:
  asp = ""
  for l in state.locations:
    asp += f"is_location(\"{l}\").\n"

  for o in state.objects:
    asp += f"is_object(\"{o.label}\", \"{o.location}\").\n"

  for p in state.interactive_agents:
    answers = ""
    for a in p.answers:
      answers += f"[{a}],"
    asp += f"is_interactive_agent(\"{p.name}\", \"{p.location}\", \"{answers}\").\n"
  return asp


program = """
list_of_rooms = get_all_rooms()
start_loc = get_current_location()
candies = [\"Chocolate\", \"Gummies\", \"Licorice\"]
candies_count = {candy : 0 for candy in candies}
for room in list_of_rooms:
  if \"office\" not in room:
    continue
  go_to(room)
  if is_in_room(\"person\"):
    response = ask(\"Person\", \"Which kind of candy would you like?\", candies)
    candies_count[response] += 1
go_to(start_loc)
for candy, count in candies_count.items():
  say(\"We need to buy \" + str(count) + \" \" + candy)
"""

# program = "say(\"Hello, world!\")"
# program = "r = 10\n"
print("=======================\nProgram:\n=======================")
print(program)
print("=======================\nState:\n=======================")
print(state)
print("=======================\nASP Trace:\n=======================")
asp_state = state_to_asp(state)
t = run_program(program, state)
# print(*t, sep="\n")
print(asp_state)
print(*asp_trace, sep="\n")