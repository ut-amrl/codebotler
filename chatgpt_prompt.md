Consider a robot capable of executing simple python programs, with the following in-built functions:

```python
# Get the current location of the robot.
def get_current_location() -> str

# Get a list of all rooms in the house.
def get_all_rooms() -> List[str]

# Check if an object is in the current room.
def is_in_room(object : str) -> bool

# Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's
# office"), go_to("Jill's study"). Immediate successive calls to go_to() can be
# optimized to avoid unnecessary movement - e.g. go_to("kitchen"); go_to("living
# room"); go_to("dining room") can be optimized to go_to("dining room").
def go_to(location : str) -> None

# Ask a person a question, and offer a set of specific options for the person to
# respond. Return with the response selected by the person.
def ask(person : str, question : str, options: Optional[List[str]]) -> str

# Say the message out loud. Make sure you are either in a room with a person, or
# at the starting location before calling this function.
def say(message : str) -> None
```

Given a natural language prompt, the task is to write a simple program that performs the task. Here are some examples:

Example 1:
Prompt: "Go to Arjun's office, ask him if he is ready to head out, and come back and tell me what he said"
Program:
```python
start_loc = get_current_location()
go_to("Arjun's office")
response = ask("Arjun", "Are you ready to go?", ["Yes", "No"])
go_to(start_loc)
say("Arjun said: " + response)
```

Example 2:
Prompt: "Go to the kitchen, ask Mom if dinner is ready, and come back and tell me what she said"
Program:
```python
start_loc = get_current_location()
go_to("kitchen")
response = ask("Mom", "Is dinner ready?", ["Yes", "No"])
go_to(start_loc)
say("Mom said: " + response)
```

Example 3:
Prompt: "Check if there are any mugs in the living room, and come back and tell me if there are any"
Program:
```python
start_loc = get_current_location()
go_to("living room")
mug_found = is_in_room("mug")
go_to(start_loc)
if mug_found:
    say("There is a mug in the living room")
else:
    say("There is no mug in the living room")
```

Example 4:
Prompt: "Check if there are any staplers in the house, and come back and tell me if there are any, and if so, where"
Program:
```python
list_of_rooms = get_all_rooms()
start_loc = get_current_location()
stapler_found = False
stapler_loc = None
for room in list_of_rooms:
    go_to(room)
    if is_in_room("stapler"):
        stapler_found = True
        stapler_loc = room
        break
go_to(start_loc)
if stapler_found:
    say("There is a stapler in the " + stapler_loc)
else:
    say("There is no stapler in the house")
```

With the above in mind, write a program that takes a natural language prompt as input, and outputs a python program that performs the task.

Prompt: "Go to the kitchen, ask Mom if dinner is ready, and come back and tell me what she said"
Program:
'''python
