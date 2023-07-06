import json
import argparse
import os
from pathlib import Path
from collections import OrderedDict
import subprocess
import re
from benchmark.evaluator.solve_utils import model_to_str
from benchmark.simple_tracer import run_program, dict_to_state
import shutil
"""
Timeout is max number of time steps after which solver is killed.
All prompted tasks should be designed to be completed
before TIMEOUT for accurate results.
"""


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
    return program + "\n\ntask_program(robot)"
    # return program


def run_simulation(program: str, state:dict,constraint: str, timeout:int, robot_asp_logic:str, debug_file:str, max_seconds = 10):
    program = code_replace(program)
    state = dict_to_state(state)
    # print(program, state)
    try:
        asp_trace = run_program(program, state)
    except Exception as e:
        raise e
        # return ("", "UNSAT")
    # print(asp_trace)

    os.makedirs("debug", exist_ok=True)
    with open(debug_file, "w") as f:
        f.write("#script (python)\n")
        f.write(open("benchmark/evaluator/solve_utils.py", 'r').read())
        f.write("#end.\n\n")
        f.write(f"#const timeout={timeout}.\n")
        f.write("\n".join(asp_trace))
        f.write(constraint)
        f.write("\n")
        f.write("\n".join([f"% {line}" for line in program.split("\n")]))

    # with open(debug_file, "r") as f:
    #     print("DEBUG:", f.read())

    # run clingo robot.lp debug/debug_ex{i+1}.lp

    out = subprocess.run(["clingo", "-f", robot_asp_logic, "-f", debug_file,
                          "--time-limit", str(max_seconds)],
                                 capture_output=True)

    if out.stderr:
        print("=====================================")
        print("Clingo error:")
        print("=====================================")
        print(out.stderr.decode("utf-8"))
        print("=====================================")
        raise Exception("Clingo error")

    if "UNSATISFIABLE" in str(out.stdout):
        return ("", "UNSAT")
    else:
        model = re.search(r"Answer: 1(.*)SATISFIABLE", str(out.stdout)).group(1)
        return (model_to_str(model.strip("\n")), "SAT")


def evaluate_trace(completions_file, eval_file, asp_file="benchmark/evaluator/robot.lp", asp_timeout=20, debug_dir="debug"):
    completions = []

    with open(Path(completions_file), 'r') as f:
        for line in f:
            completions.append(json.loads(line))

    evaluated_completions = []


    for i, example_completion in enumerate(completions):
        program = example_completion["completion"]
        for j, test in enumerate(example_completion["tests"]):
            evaluated_ex = {}
            # program, state dict, constraints
            state = eval(str(test["state"]))
            constraints = test["test"]
            (model, is_sat) = run_simulation(program,
                                            state,
                                            constraints,
                                            timeout=asp_timeout,
                                            robot_asp_logic=asp_file,
                                            debug_file=f"{debug_dir}/debug_ex{i+1}_{j+1}.lp")

            evaluated_ex["model"] = model
            evaluated_ex["is_sat"] = (is_sat == "SAT")
            print("example {}: sat is {}".format(i, evaluated_ex["is_sat"]))

            evaluated_ex["name"] = example_completion["name"]
            evaluated_ex["completion"] = program
            evaluated_ex["state"] = state
            evaluated_ex["constraint"] = constraints
            # evaluated_ex["description"] = example_completion["description"]

            # turn into ordered dict so is_sat shown first
            order = ["is_sat", "name", "state", "completion", "model", "constraint"]
            # order = ["description", "is_sat", "name", "state", "completion", "model", "constraint"]


            list_of_tuples = [(key, evaluated_ex[key]) for key in order]
            ord_evaluated_ex = OrderedDict(list_of_tuples)
            evaluated_completions.append(ord_evaluated_ex)


            with open(eval_file, "a+") as f:
                json.dump(ord_evaluated_ex, f)
                f.write("\n")


def main(args):
    evaluate_trace(args.completions_file, args.eval_file, args.asp_file, args.asp_timeout)



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('completions_file', type=str)
    parser.add_argument('eval_file', type=str)
    parser.add_argument('--asp-timeout', type=int, default=20)
    parser.add_argument('--asp-file', type=str, default="robot.lp")

    shutil.rmtree(Path("debug"))
    os.makedirs("debug", exist_ok=True)

    args = parser.parse_args()
    main(args)
