Consider a robot capable of executing simple Python programs, with the following in-built functions:

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
def ask(person : str, question : str, options: List[str]) -> str

# Say the message out loud. Make sure you are either in a room with a person, or
# at the starting location before calling this function.
def say(message : str) -> None
```

Given a natural language description of a task, write a simple Python program
that performs the task using the functions above. Do not use any functions other
than built-in Python functions and the above. Loops and conditionals must have
the appropriate tab spaces. Here are some examples:

Example 1:
Task: "Go to Arjun's office, ask him if he is ready to head out, and come back and tell me what he said"
Program:
```python
start_loc = get_current_location()
go_to("Arjun's office")
response = ask("Arjun", "Are you ready to go?", ["Yes", "No"])
go_to(start_loc)
say("Arjun said: " + response)
```

Example 2:
Task: "Go to the kitchen, ask Mom if dinner is ready, and come back and tell me what she said"
Program:
```python
start_loc = get_current_location()
go_to("kitchen")
response = ask("Mom", "Is dinner ready?", ["Yes", "No"])
go_to(start_loc)
say("Mom said: " + response)
```

Example 3:
Task: "Check if there are any mugs in the living room, and come back and tell me if there are any"
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
Task: "Check if there are any staplers in any printer room, and come back and tell me if there are any, and if so, where"
Program:
```python
list_of_rooms = get_all_rooms()
start_loc = get_current_location()
stapler_found = False
stapler_loc = None
for room in list_of_rooms:
    if "printer" not in room:
        continue
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

Example 5:
Task: "Go to the kitchen and wait for someone to show up. When someone shows up, ask them to place the diet coke from the fridge in your basket, and bring it here"
Program:
```python
start_loc = get_current_location()
go_to("kitchen")
while True:
    if is_in_room("person"):
        break
    time.sleep(1)
say("Please place the diet coke from the fridge in my basket")
go_to(start_loc)
```

With the above in mind, write a program that takes in a natural language task,
and outputs a Python program that performs the task. Make sure to preserve
necessary tabs in the output program.

Task:
