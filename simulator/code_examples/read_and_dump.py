import argparse
import json
from clingo.symbol import *

"""
This script takes a json file of example good/bad completions in
the format of staplerSupply.json and adds a constraint according to the
task name. The output is a jsonl file of the same format with the added
constraints.
"""
task_to_constraints = {
    
# Check every printer room for a stapler, and come back and tell me which ones do not have a stapler.
# Constraints:
# - only go to “printer room”
# - Don’t care the sequence of “printer rooms” being visited
# - Only check for stapler
# - Only say things about stapler and room (contains stapler, room, and no)
# - Say must be correct (contain)

    "StaplerSupply" : [
        'at("robot", "start_loc", 0).',
        'at("stapler", "printer room 1", 0).',
        'room("printer room 2").',
        # ':- not t_go_to("printer room 1", 0).',
        # ':- not t_go_to("printer room 2", 1).',
        # ':- t_go_to("conference room", _).',
        # # ':- not t_go_to("start loc", 2).',
        # ':- not t_say(_, 3).', # printer room 1
        # ':- not at("robot","start_loc", timeout).'
        
        # Assert there are no go_tos to rooms without "printer"
        ':- t_go_to(X, T), @contains_any(X, "printer", "start_loc") = 0.',
        #Check: only check stapler in room
        ':- check_at(X, _, _), X!="stapler".',
        #Check: only say things about stapler and room
        ':- t_say(X, _), @contains(X, "stapler") = 0.',
        # come back
        ':- not at("robot", "start_loc", timeout).'

    ]
}

class Temporal:
    
    def is_after(self, x0, x1, x2):
        timesteps = [x0.arguments[-1].number, 
                     x1.arguments[-1].number, 
                     x2.arguments[-1].number]
        ordered = sorted(timesteps)
        if ordered == timesteps:
            return Number(0) #true
        else:
            return Number(1)  #false

# finally [X] (x is last action, any order)

# initially [X] (x is first action in any order)

# is_after [X] (x is list of chrono actions)

# contains Y, X : check X contains string Y


def main(args):
    examples = json.loads(open(args.json_file,"r").read())
    
    with open(args.output_file,"w") as f:
        for line in examples:
            # rename program to completions
            line["completion"] = line.pop("program")
            # add constraint according to problem
            line["constraint"] = task_to_constraints[args.task_name]
            
            f.write(json.dumps(line))
            f.write("\n")
 
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('json_file', type=str)
    parser.add_argument('task_name', type=str)
    parser.add_argument('--output_file', type=str, default="constrained_examples.jsonl")
    main(parser.parse_args())