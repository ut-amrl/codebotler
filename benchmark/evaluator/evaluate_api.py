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
"""
Timeout is max number of time steps after which solver is killed. 
All prompted tasks should be designed to be completed
before TIMEOUT for accurate results.

Note: the order of rooms is NON DETERMINISTIC. Beware of tests
that can be good/bad depending on the order of rooms.
"""

def code_replace(program, sim_name):
    
    def normalize(s):
        return s.group(0).lower().replace("'", "")
    program = re.sub(r'\".*?\"', normalize, program)
    program = program.replace("wait()", f"{sim_name}.wait()")
    program = program.replace("time.sleep(", f"{sim_name}.wait(")
    program = program.replace("get_current_location(", f"{sim_name}.get_robot_location(")
    program = program.replace("get_current_loc(", f"{sim_name}.get_robot_location(")
    program = program.replace("get_all_rooms(", f"{sim_name}.get_simulation_rooms(")
    program = program.replace("is_in_room(", f"{sim_name}.is_in_robot_location(")
    program = program.replace("say(", f"{sim_name}.robot_say(")
    program = program.replace("go_to(", f"{sim_name}.robot_go_to(")
    program = program.replace("ask(", f"{sim_name}.robot_ask(")
    return program


def run_simulation(example: dict, timeout:int, robot_asp_logic:str, debug_file:str, max_seconds = 1):
    constraint = example["constraint"]
    print(debug_file)
    
    init = f"""
from solver import Context
simulator = Context(timeout={timeout}, asp_rules_file='{robot_asp_logic}', 
                    debug_file='{debug_file}')
constraints = '''{constraint}'''

simulator.add_constraints(constraints)\n
"""
    
    ret = "\n(model, is_sat) = simulator.ground_and_solve()\nprint(model, is_sat)"
    
    generated_code = code_replace(example["completion"], "simulator")
    
    result = bounded_subprocess.run(
        ["python3", "-c", init+generated_code+ret],
        timeout_seconds=max_seconds)
    
    if result.exit_code != 0:
        print("generated code failed: ", result.stderr, init+generated_code+ret) 
        is_sat = "UNSAT"
        model = ""
    elif "UNSAT" in result.stdout:
        is_sat = "UNSAT"
        model = ""
    else:
        is_sat = "SAT"
        model = result.stdout
    print(result.stdout)
    
    
    # sanity check: run clingo robot.lp debug/debug_ex{i+1}.lp
    result = bounded_subprocess.run(["clingo", "-f", "robot.lp", "-f", debug_file,  "/dev/null"], timeout_seconds = max_seconds )
    assert(("UNSATISFIABLE" not in str(result.stdout)) == (is_sat == "SAT")), str(result.stdout)+"\nis_sat:"+ is_sat
    
    # print(model, is_sat)
    return (model, is_sat) 

    # runs = []
    # model = None
    # for _ in range(n):
    #     try:
    #         exec(generated_code)
    #     except Exception as e:
    #         print("generated code failed: ", e)    
    #         return "", ""
    #     (m, is_sat) = simulator.ground_and_solve()
        
    #     # sanity check
    #     # run clingo robot.lp debug/debug_ex{i+1}.lp
    #     out = subprocess.run(["clingo", "-f", "robot.lp", "-f", debug_file,  "/dev/null"], capture_output=True)
    #     assert(("UNSATISFIABLE" not in str(out)) == (is_sat == "SAT")), str(out)+"\nis_sat:"+ is_sat
        
            
    #     runs.append(is_sat == "SAT")
    #     model = m
        
    # # if at least one unsat, is unsat
    # if all(runs):
    #     return (model, "SAT")
    # else:  
    #     return ("", "UNSAT")
    
    

def main(args):
    n = 10
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
        
        runs = []
        model = None
        for j in range(n):
            (m, is_sat) = run_simulation(example_completion, 
                                            timeout=args.asp_timeout,
                                            robot_asp_logic=args.asp_file,
                                            debug_file=f"debug/debug_ex{i+1}.lp")
            runs.append((is_sat == "SAT"))
            model = m
            
        if all(runs):
            example_completion["model"] = model
            example_completion["is_sat"] = True
            print("example {}: sat is {}".format(i, example_completion["is_sat"]))
        else:
            example_completion["model"] = ""
            example_completion["is_sat"] = False
            print("example {}: sat is {}".format(i, example_completion["is_sat"]))
        
       
         
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
    parser.add_argument('--eval_file', type=str, default="task_evaluations.jsonl")
    parser.add_argument('--asp-timeout', type=int, default=20)
    parser.add_argument('--asp-file', type=str, default="robot.lp")
    
    os.makedirs("debug", exist_ok=True)
    args = parser.parse_args()
    main(args)
