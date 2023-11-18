import typing

from misc.bounded_subprocess import run as bounded_run
import sys
from benchmark.rtl import TraceElement, SPECIAL_DELIM
from typing import List
import re
from misc.utils import (
    write_dict_to_python_file,
    write_custom_to_python_file,
    check_file_exists,
)


def code_replace(program):
    """ Translate DSL calls to interact with Python harness. """
    def normalize(s):
        return s.group(0).lower()

    program = re.sub(r"\".*?\"", normalize, program)
    sim_name = "robot"
    program = program.replace("task_program()", "task_program(robot)")
    program = program.replace(
        "get_current_location(", f"{sim_name}.get_current_location("
    )
    program = program.replace("get_all_rooms(", f"{sim_name}.get_all_rooms(")
    program = program.replace("is_in_room(", f"{sim_name}.is_in_room(")
    program = program.replace("say(", f"{sim_name}.say(")
    program = program.replace("go_to(", f"{sim_name}.go_to(")
    program = program.replace("ask(", f"{sim_name}.ask(")
    program = program.replace("pick(", f"{sim_name}.pick(")
    program = program.replace("place(", f"{sim_name}.place(")
    program = program.replace("time.sleep(1)", "time.sleep(0.01)")
    return program + "\n\ntask_program(robot)"


def print_trace_elements(trace_elements: list):
    for ele in trace_elements:
        print(ele)


def save_trace(
    save_fn,
    detailed_name,
    num_state,
    is_sat,
    trace_elements_str,
    error_message,
    program,
):
    if not check_file_exists(save_fn):
        # first time writing to file
        message = '"""field: TASKNAME_PROMPTNUM_COMPLNUM_TESTNUM"""'
        write_custom_to_python_file(message, save_fn, write_mode="w")
        write_dict_to_python_file("ordered_field_buffer", [], save_fn, write_mode="a")

    variable_name = detailed_name + f"_{num_state}"
    # recording data
    data = {
        "name": variable_name,
        "is_sat": str(is_sat),
        "trace_elements_str": trace_elements_str,
        "error_message": error_message,
    }
    write_dict_to_python_file(variable_name, data, save_fn, write_mode="a")

    program_message = f"program_{variable_name} = '''{program}'''"
    write_custom_to_python_file(program_message, save_fn, write_mode="a")

    # register to buffer
    message = f"ordered_field_buffer.append({variable_name})"
    write_custom_to_python_file(message, save_fn, write_mode="a")


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
) -> (List[TraceElement], str):
    """ Rum a program in the simulated environment. Raises an Exception if a
    Runtime Error occurs. Returns the trace and exit status of the run. """
    # TODO: there is bug here: need to further debug this
    program = code_replace(program)
    p = f"""
import sys
import time
from benchmark.simulator import Robot
from benchmark.tasks.{task_name} import tests
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


def run_execution(
    task_name: str, program: str, execution_timeout: int, execution_num: int
) -> bool:
    """ Runs a given program on a given task. Returns False if the program
    exits in an off-nominal state and True otherwise. """
    program = code_replace(program)
    p = f"""
import sys
import time
sys.path.append('../')
from benchmark.simulator import RobotExecution
from benchmark.tasks.{task_name} import tests
state = tests[0]["state"]
robot = RobotExecution(state)
{program}
"""
    for _ in range(execution_num):
        sys.stdout.flush()
        ret = bounded_run(
            ["python", "-c", p],
            timeout_seconds=execution_timeout,
            max_output_size=1000000,
        )
        if ret.exit_code != 0:
            if ret.exit_code == -1:
                ret = bounded_run(
                    ["python", "-c", p],
                    timeout_seconds=execution_timeout * 4,
                    max_output_size=1000000,
                )
                if ret.exit_code == 0:
                    continue
            return False
    return True


def evaluate_trace(completions, benchmarks, eval_file, simulation_timeout) -> None:
    """ Runs all completions in simulation and evaluates and saves the
    generated traces. """
    for i, row in completions.iterrows():
        name = row["name"]
        try:
            tests = benchmarks[benchmarks["name"] == name]["tests"].iloc[0]
        except:
            continue
        program = row["completion"]

        detailed_name = completions["detailed_name"].iloc[i]
        for num_state, test in enumerate(tests):
            constraint_cb = test["test"]
            should_timeout = test["timeout"]

            save_fn = "{}.py".format(eval_file)
            is_sat = False
            trace_elements_str = []

            try:
                # SIM
                (trace_elements, status) = run_simulation(
                    name, program, num_state, simulation_timeout
                )
                print(f"COMPLETION {i}, TEST: {num_state}, RUN SIM STATUS: {status}")
                if not should_timeout and status == "timeout":
                    raise Exception("TimeoutError: program should not timeout")

                trace_elements_str = [ele.convert_to_str() for ele in trace_elements]
                if status == "timeout":
                    trace_elements_str = trace_elements_str[:10]
            except Exception as e:
                # if sim crashed, save as False
                print("simulation program crashed: ", e)
                error_message = f"SIM: {e}"
                assert isinstance(is_sat, bool)
                save_trace(
                    save_fn,
                    detailed_name,
                    num_state,
                    is_sat,
                    trace_elements_str,
                    error_message,
                    program,
                )
                continue

            try:
                # EVAL
                is_sat = constraint_cb(trace_elements)
                print("IS SAT: ", is_sat)
                assert isinstance(is_sat, bool)
                error_message = ""
                save_trace(
                    save_fn,
                    detailed_name,
                    num_state,
                    is_sat,
                    trace_elements_str,
                    error_message,
                    program,
                )
            except Exception as e:
                # if sim crashed, save as False
                print("evaluation program crashed: ", e)
                is_sat = False
                error_message = f"EVAL: {e}"
                save_trace(
                    save_fn,
                    detailed_name,
                    num_state,
                    is_sat,
                    trace_elements_str,
                    error_message,
                    program,
                )
