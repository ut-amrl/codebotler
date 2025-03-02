messages = [
  {
    "role": "system",
    "content": """
\"\"\"Robot task programs.

Robot task programs may use the following functions:
get_current_location()
get_all_rooms()
is_in_room()
go_to(location)
ask(person, question, options)
say(message)

Robot tasks are defined in named functions, with docstrings describing the task.
\"\"\"
# Get the current location of the robot.
def get_current_location() -> str:
  ...

# Get a list of all rooms.
def get_all_rooms() -> list[str]:
  ...

# Check if an object is in the current room.
def is_in_room(object : str) -> bool:
  ...

# Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's office"), go_to("Jill's study").
def go_to(location : str) -> None:
  ...

# Ask a person a question, and offer a set of specific options for the person to respond. Returns the response selected by the person.
def ask(person : str, question : str, options: list[str]) -> str:
  ...

# Say the message out loud.
def say(message : str) -> None:
  ...
"""
  },

  {
    "role": "user",
    "content": "Go to Arjun's office, ask him if he is ready to head out, and come back and tell me what he said"
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    start_loc = get_current_location()
    go_to("Arjun's office")
    response = ask("Arjun", "Are you ready to go?", ["Yes", "No"])
    go_to(start_loc)
    say("Arjun said: " + response)
"""
  },
]