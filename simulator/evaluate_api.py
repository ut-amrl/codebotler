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
"""
Timeout is max number of time steps after which solver is killed. 
All prompted tasks should be designed to be completed
before TIMEOUT for accurate results.

Note: the order of rooms is NON DETERMINISTIC. Beware of tests
that can be good/bad depending on the order of rooms.
"""

def code_replace(program, sim_name):
    program = program.replace("get_current_loc(", f"{sim_name}.get_robot_location(")
    program = program.replace("get_all_rooms(", f"{sim_name}.get_simulation_rooms(")
    program = program.replace("is_in_room(", f"{sim_name}.is_in_robot_location(")
    program = program.replace("say(", f"{sim_name}.robot_say(")
    program = program.replace("go_to(", f"{sim_name}.robot_go_to(")
    program = program.replace("ask(", f"{sim_name}.robot_ask(")
    return program


def run_simulation(example: dict, timeout:int, robot_asp_logic:str, debug_file:str):
    
    simulator = Context(timeout=timeout, asp_rules_file=robot_asp_logic, 
                        debug_file=debug_file)
    constraints = example["constraint"]
    
    simulator.add_constraints(constraints)
    
    generated_code = code_replace(example["completion"], "simulator")
    print(generated_code)
    
    try:
        exec(generated_code)
    except Exception as e:
        print("generated code failed: ", e)    
        return "", ""
    (model, is_sat) = simulator.ground_and_solve()
    # print(model, is_sat)
    return (model, is_sat)
    
    

def main(args):

    completions = []
    with open(Path(args.completions_file), 'r') as f:
        for line in f:
            orddict = json.JSONDecoder(object_pairs_hook=collections.OrderedDict).decode(line)
            completions.append(orddict)
    
    evaluated_completions = []
    for i, example_completion in enumerate(completions):
        (model, is_sat) = run_simulation(example_completion, 
                                         timeout=args.asp_timeout,
                                         robot_asp_logic=args.asp_file,
                                         debug_file=f"debug/debug_ex{i+1}.lp")
        example_completion["model"] = model
        example_completion["is_sat"] = (is_sat == "SAT")
        print("example {}: sat is {}".format(i, example_completion["is_sat"]))
        
        # sanity check
        # run clingo robot.lp debug/debug_ex{i+1}.lp
        out = subprocess.run(["clingo", "-f", "robot.lp", "-f", f"debug/debug_ex{i+1}.lp",  "/dev/null"], capture_output=True)
        assert(("UNSAT" not in out.stdout.decode("utf-8").strip()) == example_completion["is_sat"]), str(out)+"\nis_sat:"+ example_completion["is_sat"]
        
        # to prevent future headaches:
        try:
            example_completion["description"]
        except KeyError:
            raise ValueError("Wrong format for completion file, description only used for dev")
         
        # reorder so is_sat shown first
        # orders = ("is_sat", )
        orders = ("is_sat", "description", "constraint", "completion", "model")
        for key in orders:
            v = example_completion[key]
            del example_completion[key]
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
    parser.add_argument('--eval_file', type=str, default="evaluations.jsonl")
    parser.add_argument('--asp-timeout', type=int, default=10)
    parser.add_argument('--asp-file', type=str, default="robot.lp")
    
    os.makedirs("debug", exist_ok=True)
    args = parser.parse_args()
    main(args)
