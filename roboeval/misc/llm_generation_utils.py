import re 

prefix = """\"\"\"Robot task programs.

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

def truncate_code_at_stopwords(code):
    start_length = len("def task_program():\n")
    start_idx = code.find("def task_program():\n")
    parse_code = code[start_idx+start_length:]
    min_stop_idx = len(parse_code)
    if start_idx == -1:
        return code
    
    pattern = r'\n[^\s\n]+' # anything that escapes the function
    match = re.search(pattern, parse_code)
    if match:
        min_stop_idx = re.search(pattern, parse_code).start()
    return code[start_idx:min_stop_idx + start_idx+start_length]

def post_process_llama3_program(inst):
    if inst.find("<|start_header_id|>assistant<|end_header_id|>") != -1:
        inst = inst.replace("<|start_header_id|>assistant<|end_header_id|>", "").strip()
    if inst.find("```python") != -1:
        inst = inst[inst.find("```python") + len("```python"):]
    if inst.find("```") != -1:
        inst = inst[:inst.find("```")]
    return inst

def post_process_llama3_instruction(inst):
    if inst.find("<|start_header_id|>assistant<|end_header_id|>") != -1:
        inst = inst.replace("<|start_header_id|>assistant<|end_header_id|>", "").strip()
    inst = " ".join(inst.split("\n"))
    return inst

def post_process_program(program):
    program = post_process_llama3_program(program)
    if "def task_program():\n" not in program:
        program = "def task_program():\n" + program
    program = truncate_code_at_stopwords(program)
    return program
        
def post_process_vllm_generation(outputs):
    programs = []
    for output in outputs:
        program = output.outputs[0].text
        program = post_process_program(program)
        programs.append(program)
    return programs

def construct_text_field(instruction, program):
    return prefix + "\n# Instruction: " + instruction + "\n" + program