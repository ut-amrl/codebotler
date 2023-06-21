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

# - hallucinate rooms (we wnt this)
# who cares: 13, 12, 8

    "StaplerSupply" : [
        'at("robot", "start_loc", 0).',
        'at("stapler", "printer room 1", 0).',
        'room("printer room 2").',
        'room("printer room 3").',
        'room("living room").',
        # ':- not t_go_to("printer room 1", 0).',
        # ':- not t_go_to("printer room 2", 1).',
        # ':- t_go_to("conference room", _).',
        # # ':- not t_go_to("start loc", 2).',
        # ':- not t_say(_, 3).', # printer room 1
        # ':- not at("robot","start_loc", timeout).'
        
        # Assert goes to all printer rooms at some point
        ':- not t_go_to("printer room 1", _).',
        ':- not t_go_to("printer room 2", _).',
        ':- not t_go_to("printer room 3", _).',
        # Assert there are no go_tos to rooms without "printer" (or start_loc)
        ':- t_go_to(X, T), @contains_any(X, "printer", "start_loc") = 0.',
        #Check: only check stapler in room
        ':- check_at(X, _, _), X!="stapler".',
        #Check: says something along lines of "no stapler in printer room 2" 
        ':- t_say(X, _), @contains_all(X, "stapler", "printer room 2", "not/no/n\'t") = 0.',
        #Check: says something along lines of "no stapler in printer room 3" 
        ':- t_say(X, _), @contains_all(X, "stapler", "printer room 3", "not/no/n\'t") = 0.',
        # come back
        ':- not at("robot", "start_loc", timeout).'

    ]
}

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