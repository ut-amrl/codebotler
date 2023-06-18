import sys 
import json 
import argparse
from solver import Context
import os
from pathlib import Path
"""
Timeout is max number of time steps after which solver is killed. 
All prompted tasks should be designed to be completed
before TIMEOUT for accurate results.
"""

def code_replace(program, sim_name):
    program = program.replace("get_current_loc(", f"{sim_name}.get_robot_location(")
    program = program.replace("get_all_rooms(", f"{sim_name}.get_simulation_rooms(")
    program = program.replace("is_in_room(", f"{sim_name}.is_in_robot_location(")
    program = program.replace("say(", f"{sim_name}.robot_say(")
    program = program.replace("go_to(", f"{sim_name}.robot_go_to(")
    program = program.replace("ask(", f"{sim_name}.robot_ask(")
    return program


def run_simulation(example: dict, timeout:int, robot_asp_logic:str):
    
    simulator = Context(timeout=timeout, asp_rules_file=robot_asp_logic)
    constraints = example["constraint"]
    
    simulator.add_constraints(constraints)
    
    generated_code = code_replace(example["completion"], "simulator")
    
    try:
        exec(generated_code)
    except Exception as e:
        print("generated code failed: ", e)    
        return "", ""
    (model, is_sat) = simulator.ground_and_solve()
    return (model, str(is_sat))
    
  
TIMEOUT = 10
ROBOT_ASP_LOGIC = "robot.lp"  

def main(completions_file: str):

    completions = []
    with open(Path(completions_file), 'r') as f:
        for line in f:
            completions.append(json.loads(line))
    
    evaluated_completions = []
    for i, example_completion in enumerate(completions):
        (model, is_sat) = run_simulation(example_completion, 
                                         timeout=TIMEOUT,
                                         robot_asp_logic=ROBOT_ASP_LOGIC)
        example_completion["model"] = model
        example_completion["is_sat"] = (is_sat == "SAT")
        print("example {}: sat is {}".format(i, example_completion["is_sat"]))
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
    args = parser.parse_args()
    main(args.completions_file)
