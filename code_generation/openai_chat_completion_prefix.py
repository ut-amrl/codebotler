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
pick(object)
place(object)

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

# Pick up an object if you are not already holding one. You can only hold one object at a time.
def pick(obj: str) -> None:
    ...

# Place an object down if you are holding one.
def place(obj: str) -> None:
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
    "content": "Ask Alice if she needs 1, 2, or 3 boxes. Go to the storage room and ask if they have that many boxes. If so, go place the boxes in Alice's office. Otherwise, tell Alice you could not get the boxes."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("Alice's office")
    num_boxes = ask("Alice", "How many boxes do you need?", ["1", "2", "3"])
    go_to("storage room")
    response = ask("", "Do you have" + num_boxes + " boxes?", ["Yes", "No"])
    if response == "Yes":
        for _ in range(int(num_boxes)):
            pick("box")
            go_to("Alice's office")
            place("box")
            go_to("storage room")
    else:
        go_to("Alice's office")
        say("I could not get the boxes")
"""
  },

  {
    "role": "user",
    "content": "Check if there is a red marker in the main office, and if so, tell Eve that there is a marker there. If not, go to the supply room and bring a red marker to the main office."
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
        pick("red marker")
        go_to("main office")
        place("red marker")
"""
  },

  {
    "role": "user",
    "content": "Check every classroom if there is a whiteboard. Go to Aiden's office to tell him which room does not have a whiteboard. Come back and tell me task is completed."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    start_loc = get_current_location()
    list_of_rooms = get_all_rooms()
    room_without_whiteboard = []
    for room in list_of_rooms:
        if "classroom" not in room:
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
    "content": "Go to the kitchen and wait for someone to show up. When someone shows up, ask them to open the fridge, then pick up a diet coke. Finally, put the diet coke in the living room."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("kitchen")
    while True:
        if is_in_room("person"):
            response = ask("", "Please open the fridge", ["Yes", "No"])
            if response == "Yes":
                pick("diet coke")
                break
        time.sleep(1)
    go_to("living room")
    place("diet coke")
"""
  }, 

  {
    "role": "user",
    "content": "Take a bed sheet from the laundry room and put it in each of the bedrooms."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    start_loc = get_current_location()
    list_of_rooms = get_all_rooms()
    for room in list_of_rooms:
        if "bedroom" not in room:
            continue
        go_to("laundry room")
        pick("bed sheet")
        go_to(room)
        place("bed sheet")
    go_to(start_loc)
"""
  }, 
]
