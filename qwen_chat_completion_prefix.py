messages = [
  {
    "role": "system",
    "content": """
\"\"\"Robot task programs.

Robot task programs may use the following functions:
go_to(location)
say(message)
pick(object)
place(object)

Robot tasks are defined by one function named task_program.

Output format rules:
- Return raw Python source only.
- Do not wrap the answer in Markdown or ```python fences.
- Do not include explanations, comments before the code, or prose after the code.
- The entire answer must be exactly one top-level function named task_program.
- Use only the robot task helper functions listed here.
- Do not define helper functions, classes, imports, or global variables.
- Use four spaces for indentation.
\"\"\"
# Go to a specific named location, e.g. go_to("kitchen"), go_to("Arjun's office"), go_to("Jill's study").
def go_to(location : str) -> None:
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
    "content": "Go to Arjun's office and pick up the book, then bring it to the kitchen."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("Arjun's office")
    pick("book")
    go_to("kitchen")
    place("book")
"""
  },

  {
    "role": "user",
    "content": "Go to the supply room, pick up a stapler, and bring it to Alice's office."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("supply room")
    pick("stapler")
    go_to("Alice's office")
    place("stapler")
"""
  },

  {
    "role": "user",
    "content": "Tell everyone in the main office that the meeting starts in 10 minutes."
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("main office")
    say("The meeting starts in 10 minutes")
"""
  },

  {
    "role": "user",
    "content": "I am thirsty"
  },
  {
    "role": "assistant",
    "content": """
def task_program():
    go_to("kitchen")
    pick("water bottle")
    go_to("office")
    place("water bottle")
    say("Here is some water")
"""
  },

]