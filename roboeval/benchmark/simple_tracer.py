from roboeval.misc.bounded_subprocess import run as bounded_run
from roboeval.benchmark.rtl import TraceElement, SPECIAL_DELIM
from roboeval.misc.utils import read_benchmark

import sys
import re
import numpy as np
from typing import List, Union

def code_replace(program):
    def normalize(s):
        return s.group(0).lower()

    program = re.sub(r'\".*?\"', normalize, program)
    sim_name = "robot"
    program = program.replace("task_program()", "task_program(robot)")
    program = program.replace("get_current_location(", f"{sim_name}.get_current_location(")
    program = program.replace("get_all_rooms(", f"{sim_name}.get_all_rooms(")
    program = program.replace("is_in_room(", f"{sim_name}.is_in_room(")
    program = program.replace("say(", f"{sim_name}.say(")
    program = program.replace("go_to(", f"{sim_name}.go_to(")
    program = program.replace("ask(", f"{sim_name}.ask(")
    program = program.replace("pick(", f"{sim_name}.pick(")
    program = program.replace("place(", f"{sim_name}.place(")
    def replace_sleep(match):
        original_argument = match.group(1)  # Capture the original argument
        # Append the multiplication by 0 to the entire argument
        new_argument = f"({original_argument}) * 0"
        return f'time.sleep({new_argument})'

    # Regex to match time.sleep() with any arguments, assuming there's no nested parentheses
    pattern = r'time\.sleep\(((?:[^()]+|\((?:[^()]+|\([^()]*\))*\))*)\)'
    program = re.sub(pattern, replace_sleep, program)
    return program + "\n\ntask_program(robot)"

def construct_trace_element_from_str(s: str) -> List[TraceElement]:
    """ Parse string into a sequence of TraceElements. """
    result = []
    s = s.strip().split("\n")

    if len(s[0]) == 0:
        return result
    for line in s:
        try:
            splitted_line = line.split(SPECIAL_DELIM)
            if len(splitted_line) != 3:
                break
            name = splitted_line[0].strip()
            value1 = splitted_line[1].strip()
            value2 = eval(splitted_line[2].strip())
            result.append(TraceElement(name, value1, value2))
        except Exception as e:
            break
    return result

def run_simulation(
    task_name: str, program: str, state_idx: int, simulation_timeout: int
) -> Union[List[TraceElement], str]:
    """ Rum a program in the simulated environment. Raises an Exception if a
    Runtime Error occurs. Returns the trace and exit status of the run. """
    # TODO: there is bug here: need to further debug this
    program = code_replace(program)
    p = f"""
import sys
import time
from roboeval.benchmark.simulator import Robot
from roboeval.benchmark.tasks.{task_name} import tests
state = tests[{state_idx}]["state"]
robot = Robot(state)
{program}
"""
    sys.stdout.flush()
    ret = bounded_run(
        ["python", "-c", p], timeout_seconds=simulation_timeout, max_output_size=1000000
    )

    if ret.exit_code == -1:
        print("PYTHON TIMED_OUT:", ret.exit_code)
        trace_elements = construct_trace_element_from_str(ret.stdout)
        return trace_elements, "timeout"
    elif ret.exit_code == 0:
        print("Program Successful")
        trace_elements = construct_trace_element_from_str(ret.stdout)
        return trace_elements, "success"
    else:
        print("PYTHON RUNTIME ERROR: ", ret.exit_code)
        py_error = ret.stderr.strip("\n").split("\n")[-1].strip()
        raise Exception(py_error)
      
def evaluate_program(taskname, program, tests) -> Union[bool, str]:
    """
        return True if the program passes all the tests
        else return False and an error type
    """
    return_val = True
    for num_state, test in enumerate(tests):
        constraint_cb = test["test"]
        should_timeout = test["timeout"]
        try:
            (trace_elements, status) = run_simulation(taskname, program, num_state, 1)
            if not should_timeout and status == "timeout":
                return False, "PythonError"
            
            is_sat = constraint_cb(trace_elements)
            if not is_sat:
                return_val = False
        except Exception as e:
            if "Robot" in str(e):
                return False, "RobotExecutionError"
            else:
                return False, "PythonError"
    
    if return_val:
        return return_val, "Success"
    else:
        return return_val, "CompletionError"
      
def evaluate_task(benchmark_file, programs, taskname, variation_idx):
    benchmark_tasks = read_benchmark(benchmark_file, "*")
    task = benchmark_tasks[benchmark_tasks["name"] == taskname]
    tests = task["tests"].iloc[0]

    result = {
        f"{taskname}_{variation_idx}" : [],
        "error_names": []
    }
    for _, program in enumerate(programs):
        is_sat, error_name = evaluate_program(taskname, program, tests)
        result[f"{taskname}_{variation_idx}"].append(int(is_sat))
        result["error_names"].append(error_name)
    
    for key in result:
        if key != "error_names":
            result[key] = np.mean(result[key])
    return result