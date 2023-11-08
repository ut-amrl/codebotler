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

  {
    "role": "user",
    "content": "Ask Alice if she needs 1, 2, or 3 staplers, and then go to the supply room and tell them how many she needs."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("Alice's office")
    response = ask("Alice", "How many staplers do you need?", ["1", "2", "3"])
    go_to("supply room")
    say("Alice needs " + str(response) + " staplers")
"""
  },

  {
    "role": "user",
    "content": "Check if there is a red marker in the main office, and if so, tell Eve that there is a marker there. If not, go to the supply room and tell them that the main office needs a red marker."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("main office")
    red_marker_found = is_in_room("red marker")
    if red_marker_found:
        go_to("Eve's office")
        say("There is a red marker in the main office")
    else:
        go_to("supply room")
        say("The main office needs a red marker")
"""
  },

  {
    "role": "user",
    "content": "Check all classrooms if there is a whiteboard. Go to Aiden's office to tell him which room does not have a whiteboard. Finally, come back and tell me task is completed."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    start_loc = get_current_location()
    list_of_rooms = get_all_rooms()
    room_without_whiteboard = []
    for room in list_of_rooms:
        if "classrooms" not in room:
            continue
        go_to(room)
        if not is_in_room("whiteboard"):
            room_without_whiteboard.append(room)
    go_to("Aiden's office")
    if len(room_without_whiteboard) > 0:
        message = ""
        for room in room_without_whiteboard:
            message += room + ", "
        message += "do not have a whiteboard"
    else:
        message = "all classrooms have a whiteboard"
    say(message)
    go_to(start_loc)
    say("task is completed")
"""
  },

  {
    "role": "user",
    "content": "Go to the kitchen and wait for someone to show up. When someone shows up, ask them to place the diet coke from the fridge in your basket, and bring it here"
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    start_loc = get_current_location()
    go_to("kitchen")
    while True:
        if is_in_room("person"):
            response = ask("", "Could you please place the diet coke from the fridge in my basket?", ["Yes", "No"])
            if response == "Yes":
                break
        time.sleep(1)
    go_to(start_loc)
"""
  }, 
]