import argparse
import json
from clingo.symbol import *
import re
"""
This script takes a jsonl file as input and inserts the state/constraints
for the respective tasks, then dumps the modified file to a new output file.
"""

# load constrains from files
subdir = "benchmark/evaluator/constraints"
movie_messenger = re.split(r"#external state\(\d\).", open(f"{subdir}/MovieMessenger.lp", "r").read())
stapler_supply = re.split(r"#external state\(\d\).", open(f"{subdir}/StaplerSupply.lp", "r").read())
halloween_list = re.split(r"#external state\(\d\).", open(f"{subdir}/HalloweenList.lp", "r").read())
lunch_break = re.split(r"#external state\(\d\).", open(f"{subdir}/LunchBreak.lp", "r").read())
elevator_tour = re.split(r"#external state\(\d\).", open(f"{subdir}/ElevatorTour.lp", "r").read())

movie_messenger_state_0 = {
    "locations": [ "sally's office", "mark's office" ],
    "interactive_agents" : [
        # 1. Sally says "yes, I want to go to movie!"
        { "name" : "sally", "location" : "sally's office", "answers" : ["yes"]},
        # 2. Mark says 5pm
        { "name" : "mark", "location" : "mark's office", "answers" : ["5pm"]},
    ],
    "objects": [],
    "additional_constraints" : movie_messenger[1],
    "robot_location": "sally's office"
}

halloween_list_state_0 = {
    "locations": [ "sally's office", "mark's office", "cindy's office", "start_loc" ],
    "interactive_agents" : [
        # sally and choices"
        { "name" : "person", "location" : "sally's office", "answers" : ["chocolate"]},
        { "name" : "", "location" : "sally's office", "answers" : ["chocolate"]}, #diff way of writing person
        # 2. mark and choices
        { "name" : "person", "location" : "mark's office", "answers" : ["chocolate"]},
        { "name" : "", "location" : "mark's office", "answers" : ["chocolate"]}, #diff way of writing person

    ],
    "objects": [],
    "robot_location": "start_loc",
    "additional_constraints": halloween_list[1]
}

halloween_list_state_1 = {
    "locations": [ "sally's office", "mark's office", "cindy's office", "start_loc" ],
    "interactive_agents" : [
        # sally and choices"
        { "name" : "person", "location" : "sally's office", "answers" : ["chocolate"]},
        { "name" : "", "location" : "sally's office", "answers" : ["chocolate"]},
        # 2. mark and choices
        { "name" : "person", "location" : "mark's office", "answers" : ["gummy"]},
        { "name" : "", "location" : "mark's office", "answers" : ["gummy"]},
    ],
    "objects": [],
    "robot_location": "start_loc",
    "additional_constraints": halloween_list[2]
}

stapler_supply_state_0 = {
        "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
        "objects": [{ "label": "stapler", "location": "printer room 1" }], 
        "interactive_agents": [], 
        "robot_location": "start_loc",
        "additional_constraints" : stapler_supply[1]
        }

stapler_supply_state_1 = {
        "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
        "objects": [{ "label": "stapler", "location": "living room" }], 
        "interactive_agents": [], 
        "robot_location": "start_loc",
        "additional_constraints" : stapler_supply[2]
        }

stapler_supply_state_2 = {
        "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
        "objects": [], 
        "interactive_agents": [], 
        "robot_location": "start_loc",
        "additional_constraints" : stapler_supply[3]
        }

lunch_break_state_0 = {
        "locations" : ["alice's office", "bob's office", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["yes"]},
                                {"name" : "bob", "location" : "bob's office", "answers" : ["yes"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : lunch_break[1]
        }

lunch_break_state_1 = {
        "locations" : ["alice's office", "bob's office", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["no"]},
                                {"name" : "bob", "location" : "bob's office", "answers" : ["no"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : lunch_break[2]
    }

lunch_break_state_2 = {
        "locations" : ["alice's office", "bob's office", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["yes"]},
                                {"name" : "bob", "location" : "bob's office", "answers" : ["no"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : lunch_break[3]
        }

lunch_break_state_3 = {
        "locations" : ["alice's office", "bob's office", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["no"]},
                                {"name" : "bob", "location" : "bob's office", "answers" : ["yes"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : lunch_break[4]
        }
        
elevator_tour_state_0 = {
            "locations" : ["elevator", "main conference room", "start_loc"],
            "objects" : [],
            "interactive_agents" : [{"name" : "person", "location" : "elevator", "answers" : ["yes"]}, # diff way of writing same person
                                    {"name" : "", "location" : "elevator", "answers" : ["yes"]}],
            "robot_location" : "start_loc",
            "additional_constraints" : elevator_tour[1]
        }

elevator_tour_state_1 = {
            "locations" : ["elevator", "main conference room", "start_loc"],
            "objects" : [],
            "interactive_agents" : [{"name" : "person", "location" : "elevator", "answers" : ["no"]}, # diff way of writing same person
                                    {"name" : "", "location" : "elevator", "answers" : ["no"]}],
            "robot_location" : "start_loc",
            "additional_constraints" : elevator_tour[2]
        }

elevator_tour_state_2 = {
            "locations" : ["elevator", "main conference room", "start_loc"],
            "objects" : [],
            "interactive_agents" : [{"name" : "person", "location" : "start_loc", "answers" : ["yes"]}, # diff way of writing same person
                                    {"name" : "", "location" : "start_loc", "answers" : ["yes"]}],
            "robot_location" : "start_loc",
            "additional_constraints" : elevator_tour[3]
        }

task_to_states = {

    "StaplerSupply" : [stapler_supply_state_0, stapler_supply_state_1, stapler_supply_state_2],
    "LunchBreak" : [lunch_break_state_0, lunch_break_state_1, lunch_break_state_2, lunch_break_state_3],
    "ElevatorTour" : [elevator_tour_state_0, elevator_tour_state_1, elevator_tour_state_2],
    "MovieMessenger": [ movie_messenger_state_0 ],
    "HalloweenList": [halloween_list_state_0, halloween_list_state_1]
}


task_to_constraints = {
    
    "StaplerSupply" : stapler_supply[0],
    "LunchBreak" : lunch_break[0],
    "ElevatorTour" : elevator_tour[0],
    "MovieMessenger" : movie_messenger[0],
    "HalloweenList" : halloween_list[0],
}

def constrain_jsonl(args):
    with open(args.input_jsonl_file,"r") as f:
        with open(args.output_file,"a") as o:
            for line in f.readlines():
                # add constraints per state
                # only for target tests
                # list of {state, test}
                line = json.loads(line)
                task_name = line["name"].split("-")[0]
                if task_name in task_to_states.keys():
                    tests = []
                    for state in task_to_states[task_name]:
                        state_mod = {k:v for k,v in state.items() if k != "additional_constraints"}
                        
                        test = {"state" : state_mod, 
                              "test" : task_to_constraints[task_name] + "\n" + state["additional_constraints"]} 
                        tests.append(test)
                        
                    line["tests"] = tests
                    # dump
                    o.write(json.dumps(line))
                    o.write("\n")
            
def main(args):
    constrain_jsonl(args)
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('input_jsonl_file', type=str)
    parser.add_argument('output_file', type=str)
    args = parser.parse_args()
    main(args)