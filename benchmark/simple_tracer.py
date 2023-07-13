import typing
import bounded_subprocess
import sys

'''
Note: this prints to stdout to create trace (bounded subproc)
'''

def dict_to_state(state_dict):
    return State(
    locations= state_dict["locations"],
    objects = [Object(label = d["label"], location=d["location"]) for d in state_dict["objects"]],
    interactive_agents = [
      InteractiveAgent(name = agent["name"],
                      location = agent["location"],
                      answers = agent["answers"])
      for agent in state_dict["interactive_agents"]
    ],
    robot_location = state_dict["robot_location"],
  )
    
class Object:
  def __init__(self, label : str, location : str):
    self.label = label
    self.location = location

  def __str__(self) -> str:
    return "{ \"label\": \"" + self.label + "\", \"location\": \"" + \
        self.location + "\" }"

  def __repr__(self) -> str:
    return "{ \"label\": \"" + self.label + "\", \"location\": \"" + \
        self.location + "\" }"

class InteractiveAgent:
  def __init__(self, name : str, location : str, answers : list[str]) :
    self.name = name
    self.location = location
    self.answers = answers

  def __str__(self) -> str:
    return "{ \"name\": \"" + self.name + \
        "\", \"location\": \"" + self.location + "\", \"answers\": " + \
        str(self.answers) + " }"

  def __repr__(self) -> str:
    return "{ \"name\": \"" + self.name + \
        "\", \"location\": \"" + self.location + "\", \"answers\": " + \
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
        "\"locations\": " + str(self.locations) + ", \n" + \
        "\"objects\": " + str(self.objects) + ", \n" + \
        "\"interactive_agents\": " + str(self.interactive_agents) + ", \n" + \
        "\"robot_location\": \"" + str(self.robot_location) + "\"\n}"

class Robot:
  def __init__(self, state : State):
    self.state = state
    self.trace_t = 0
    self.asp_trace : list[str] = []
    self.add_world_state()
    
  def add_world_state(self):
    for locations in self.state.locations:
      self.asp_trace.append(f"room(\"{locations}\").")
      print(f"room(\"{locations}\").", flush=True)
    for objects in self.state.objects:
      self.asp_trace.append(f"at(\"{objects.label}\",\"{objects.location}\", 0).")
      print(f"at(\"{objects.label}\",\"{objects.location}\", 0).", flush=True)
    for agents in self.state.interactive_agents:
      self.asp_trace.append(f"at(\"{agents.name}\",\"{agents.location}\", 0).")
      print(f"at(\"{agents.name}\",\"{agents.location}\", 0).", flush=True)
    self.asp_trace.append(f"at(\"robot\",\"{self.state.robot_location}\", 0).")
    print(f"at(\"robot\",\"{self.state.robot_location}\", 0).", flush=True)
      
  # Get the current location of the robot.
  def get_current_location(self) -> str :
    self.asp_trace.append(f"t_get_current_location({self.trace_t}).")
    print(f"t_get_current_location({self.trace_t}).", flush=True)
    self.trace_t += 1
    return self.state.robot_location

  # Get a list of all rooms in the house.
  def get_all_rooms(self) -> list[str] :
    self.asp_trace.append(f"t_get_all_rooms({self.trace_t}).")
    print(f"t_get_all_rooms({self.trace_t}).", flush=True)
    self.trace_t += 1
    return self.state.locations

  # Check if an entity (person or object) is in the current room.
  def is_in_room(self, entity : str) -> bool :
    self.asp_trace.append(f"t_is_in_room(\"{entity}\",{self.trace_t}).")
    print(f"t_is_in_room(\"{entity}\",{self.trace_t}).", flush=True)
    self.trace_t += 1
    for o in self.state.objects:
      if o.location == self.state.robot_location and o.label == entity:
        return True
    for a in self.state.interactive_agents:
      if a.location == self.state.robot_location and a.name == entity:
        return True

    return False

  # Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
  # office"), go_to("Jill's study").
  def go_to(self, location : str) -> None :
    self.asp_trace.append(f"t_go_to(\"{location}\",{self.trace_t}).")
    print(f"t_go_to(\"{location}\",{self.trace_t}).", flush=True)
    self.trace_t += 1
    global state
    # assert location in self.state.locations, location
    # if location not in self.state.locations:
    #   self.asp_trace.append(":- .") # unsat
    self.state.robot_location = location
    # print(state)

  # Ask a person a question, and offer a set of specific options for the person to
  # respond. Return with the response selected by the person.
  def ask(self, person : str, question : str, options: list[str]) -> str :
    options_str = ""
    for o in options:
      self.asp_trace.append(f"option(\"{o}\").")
      print(f"option(\"{o}\").", flush=True)
      options_str += f"[{o}],"
    options_str = options_str[:-1]
    self.asp_trace.append(f"t_ask(\"{person}\", \"{question}\", \"{options_str}\", {self.trace_t}).")
    print(f"t_ask(\"{person}\", \"{question}\", \"{options_str}\", {self.trace_t}).", flush=True)
    self.trace_t += 1
    for p in self.state.interactive_agents:
      # if p.location == state.robot_location and p.name == person:
      if p.location == self.state.robot_location:
        # print(f"matched location {state.robot_location}")
        for a in p.answers:
          for o in options:
            if o == a:
              response = a
              self.asp_trace.append(f"reply(\"{person}\", \"{response}\",{self.trace_t}).")
              print(f"reply(\"{person}\", \"{response}\",{self.trace_t}).", flush=True)
              return response
        # print(f"no match between {options} and {p.answers}")
        # No matching answer found.
        return options[0]
    # No matching person found at the location.
    return options[0]

  # Say the message out loud. Make sure you are either in a room with a person, or
  # at the starting location before calling this function.
  def say(self, message : str) -> None :
    self.asp_trace.append(f"t_say(\"{message}\",{self.trace_t}).")
    print(f"t_say(\"{message}\",{self.trace_t}).", flush=True)
    self.trace_t += 1
    pass

def run_program(program : str, state : State) -> list[str] :
              
  p = f"""
import sys
import time
from benchmark.simple_tracer import Robot, dict_to_state
state_dict = {state}
state = dict_to_state(state_dict)
robot = Robot(state)\n{program}\n
"""
  sys.stdout.flush()
  ret = bounded_subprocess.run(["python", "-c", p], timeout_seconds=3)
  sys.stdout.flush()
  
  asp_trace = [i for i in ret.stdout.split("\n") if i != ""]
  if ret.exit_code == -1:
    print("PYTHON TIMED_OUT:", ret.exit_code)
    # crop last line of trace
    return asp_trace[:-1] + ["python_timed_out."]
  elif ret.exit_code == 0:
    
    assert len(asp_trace) > 0, p+ "\n".join(asp_trace)
    print(asp_trace[-1])
    return asp_trace
  else:
    print("PYTHON RUNTIME ERROR: ", ret.exit_code)
    py_error = ret.stderr.strip("\n").split("\n")[-1].strip()
    # asp_trace.append("""python_runtime_error(" """ + py_error + """ ").""")
    return ["""python_runtime_error(" """ + py_error + """ ")."""]


# program = """
# list_of_rooms = get_all_rooms()
# start_loc = get_current_location()
# candies = [\"Chocolate\", \"Gummies\", \"Licorice\"]
# candies_count = {candy : 0 for candy in candies}
# for room in list_of_rooms:
#   if \"office\" not in room:
#     continue
#   go_to(room)
#   if is_in_room(\"person\"):
#     response = ask(\"Person\", \"Which kind of candy would you like?\", candies)
#     candies_count[response] += 1
# go_to(start_loc)
# for candy, count in candies_count.items():
#   say(\"We need to buy \" + str(count) + \" \" + candy)
# """

# def main():
#   state = State(
#     locations=["Arjun's office",
#                 "Kitchen",
#                 "Joydeep's office",
#                 "Conference room"],
#     objects = [
#       Object(label = "pringles", location = "Kitchen" ),
#       Object(label = "black backpack", location = "Conference room"),
#       Object(label = "person", location = "Arjun's office"),
#       Object(label = "person", location = "Joydeep's office")
#       ],
#     interactive_agents = [
#       InteractiveAgent(name = "Arjun",
#                       location = "Arjun's office",
#                       answers = ["yes", "5", "Gummies"]),
#       InteractiveAgent(name = "Arjun",
#                       location = "Arjun's office",
#                       answers = ["no", "1"]),
#       InteractiveAgent(name = "Joydeep",
#                       location = "Conference room",
#                       answers = ["no", "Licorice"])
#     ],
#     robot_location = "Arjun's office"
#   )
#   program = "say(\"Hello, world!\")"
#   program = "r = 10\n"
#   print("=======================\nProgram:\n=======================")
#   print(program)
#   print("=======================\nState:\n=======================")
#   print(state)
#   asp_trace = run_program(program, state)
#   print("=======================\nASP Trace:\n=======================")
#   print(*asp_trace, sep="\n")

# if __name__ == "__main__":
#   main()