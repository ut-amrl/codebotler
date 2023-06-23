import sys 
import json 
import argparse
from solver import Context
import os
from pathlib import Path
import collections
from clingo import Control
from solve_utils import model_to_str
import subprocess
import bounded_subprocess
import re
import sys
sys.path.append("..")
from simple_tracer import run_program, State, InteractiveAgent, Object

"""
Timeout is max number of time steps after which solver is killed. 
All prompted tasks should be designed to be completed
before TIMEOUT for accurate results.

Note: the order of rooms is NON DETERMINISTIC. Beware of tests
that can be good/bad depending on the order of rooms.
"""

def dict_to_state(state_dict):
    return State(
    locations= state_dict["locations"],
    objects = [Object(label = d["label"], location=d["location"]) for d in state_dict["objects"]],
    interactive_agents = [
      InteractiveAgent(name = agent["name"],
                      location = agent["location"],
                      answers = agent["answers"])
      for agent in state_dict["interactive_agents"]
    ],
    robot_location = state_dict["robot_location"],
  )

def code_replace(program):
    
    def normalize(s):
        return s.group(0).lower().replace("'", "")
    program = re.sub(r'\".*?\"', normalize, program)
    return program


def run_simulation(example: dict, timeout:int, robot_asp_logic:str, debug_file:str, max_seconds = 10):
    constraint = example["constraint"]
    program = code_replace(example["completion"])
    state = dict_to_state(eval(example["state"]))
    try:
        asp_trace = run_program(program, state)
    except Exception as e:
        return ("", "UNSAT")
    
    with open(debug_file, "w") as f:
        f.write("#script (python)\n")
        f.write(open("solve_utils.py", 'r').read())
        f.write("#end.\n\n")
        f.write(f"#const timeout={timeout}.\n")
        f.write("\n".join(asp_trace))
        f.write(constraint)
        
    # run clingo robot.lp debug/debug_ex{i+1}.lp
    out = subprocess.run(["clingo", "-f", robot_asp_logic, "-f", debug_file, "--time-limit", str(max_seconds)], 
                                 capture_output=True)

    
    if "UNSATISFIABLE" in str(out.stdout):
        return ("", "UNSAT")
    else:
        model = re.search(r"Answer: 1(.*)SATISFIABLE", str(out.stdout)).group(1)
        return (model_to_str(model.strip("\n")), "SAT")
    
    

def main(args):
    completions = []
    with open(Path(args.completions_file), 'r') as f:
        for line in f:
            orddict = json.JSONDecoder(object_pairs_hook=collections.OrderedDict).decode(line)
            completions.append(orddict)
    
    evaluated_completions = []
    
    
    for i, example_completion in enumerate(completions):

         # to prevent future headaches:
        try:
            example_completion["description"]
        except KeyError:
            raise ValueError("Wrong format for completion file, description only used for dev")
        
        (model, is_sat) = run_simulation(example_completion, 
                                            timeout=args.asp_timeout,
                                            robot_asp_logic=args.asp_file,
                                            debug_file=f"debug/debug_ex{i+1}.lp")
        
        example_completion["model"] = model
        example_completion["is_sat"] = (is_sat == "SAT")
        print("example {}: sat is {}".format(i, example_completion["is_sat"]))
        
         
        # reorder so is_sat shown first
        # orders = ("is_sat", )
        order = ("is_sat", "description", "constraint", "state", "completion", "model")
        for key in order:
            v = example_completion[key]
            del(example_completion[key])
            example_completion[key] = v
    
        evaluated_completions.append(example_completion)
        
    
    with open(args.eval_file, "a+") as f:
        for comp in evaluated_completions:
            # json.dump(comp, f, indent=4)
            json.dump(comp, f)
            f.write("\n")
    
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('completions_file', type=str)
    parser.add_argument('--eval_file', type=str, default="task_evaluations.jsonl")
    parser.add_argument('--asp-timeout', type=int, default=10)
    parser.add_argument('--asp-file', type=str, default="robot.lp")
    
    os.makedirs("debug", exist_ok=True)
    args = parser.parse_args()
    main(args)
