import argparse
import json
from clingo.symbol import *
import sys
sys.path.append("..")
from simple_tracer import State, Object, InteractiveAgent
"""
This script takes a json file of example good/bad completions in
the format of staplerSupply.json and adds a constraint according to the
task name. The output is a jsonl file of the same format with the added
constraints.
"""
task_to_states = {

    "staplersupply1" : {
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [{ "label": "stapler", "location": "printer room 1" }], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    "additional_constraints" : ""
    },
    "staplersupply2" : {
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [{ "label": "stapler", "location": "living room" }], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    "additional_constraints" : ""
    },
    "staplersupply3" : {
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    "additional_constraints" : ""
    },
    "lunchbreak1" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["yes"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["yes"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- not joining(\"bob\").\n:- not joining(\"alice\")."
    },
    "lunchbreak2" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["no"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["no"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- joining(\"bob\").\n:- joining(\"alice\")."
    },
    "lunchbreak3" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["yes"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["no"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- joining(\"bob\").\n:- not joining(\"alice\")."
    },
    "lunchbreak4" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["no"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["yes"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- not joining(\"bob\").\n:- joining(\"alice\")."
    },
}

task_to_constraints = {
    
    "staplersupply" : '''

:- not t_go_to("printer room 1", _).
:- not t_go_to("printer room 2", _).
:- not t_go_to("printer room 3", _).
:- t_go_to(X, T), @contains_any(X, "printer", "start_loc") = 0.
:- t_is_in_room(X, _), X!="stapler".
 
:- t_say(X, _), @contains_all(X, "stapler", "printer room 2", "not/no/dont") = 0.

:- t_say(X, _), @contains_all(X, "stapler", "printer room 3", "not/no/dont") = 0.
:- t_say(X, _), @contains_all(X, "living", "not/no/dont") = 1.
:- not at("robot", "start_loc", timeout).

:- not t_go_to(_,_).
:- not t_say(_,_).
:- not t_is_in_room(_,_).
        ''',
        
    "lunchbreak" : '''
both_no :- replied("alice","no",_), replied("bob","no",_).
both_yes :- replied("alice","yes",_), replied("bob","yes",_).
             
alice_yes :- replied("alice","yes",_).
alice_no :- replied("alice","no",_).

bob_yes :- replied("bob","yes",_).
bob_no :- replied("bob","no",_).

:- alice_no, alice_yes.
:- bob_no, bob_yes.
:- not alice_no, not alice_yes.
:- not bob_no, not bob_yes.

:- not t_is_in_room("alice", _).
:- not t_is_in_room("bob", _).

:- t_go_to(X, T), @contains_any(X, "alice", "bob", "start_loc") = 0.

:- not t_ask("alice",_,_,_).
:- not t_ask("bob",_,_,_).
:- not replied(_,_,_).
:- not at("robot","start_loc", timeout).

:- replied(_,"yes",T), t_say(X, T), @contains_all(X, "meet", "lobby", "5 minutes/five minutes") = 0.
:- replied(_,"no",T), t_say(X, T), @contains_all(X, "meet", "lobby", "5 minutes/five minutes") = 1.
report_joining(P, T) :- replied(P, "yes", _), t_say(X, T), @contains_all(X, P, "joining/join/come/coming","lunch") = 1.
joining(P) :- report_joining(P, _).
:- report_joining(P, T), not at("robot", "start_loc", T).

:- option(X), @contains_any(X, "yes", "no") = 0.
''',

# at("robot", "start_loc", 0).
# at("person", "elevator", 4).
# room("conference_room").

    "elevatortour" : '''

:- t_go_to(X, T), @contains_any(X, "elevator", "start_loc") = 0.

robot_welcome :- at("robot", "elevator", T), at(P, "elevator", T), P != "robot",
    t_say(X, T), @contains_all(X, "welcome", "university") = 1.
    
:- not robot_welcome.
:- not at("robot", "conference_room", timeout).

robot_enjoy_visit :- at("robot", "conference_room", T), 
                t_say(X, T), @contains_all(X, "enjoy", "visit") = 1.
:- not robot_enjoy_visit.
'''
}


def main(args):
    examples = json.loads(open(args.json_file,"r").read())
    
    with open(args.output_file,"w") as f:
        for line in examples:
            # rename program to completions
            line["completion"] = line.pop("program")
            # add constraint according to problem
            line["constraint"] = task_to_constraints[args.task_name]
            
            list_of_states = [(k,v) for k,v in task_to_states.items() 
                              if args.task_name in k.lower()]
            
            for (problem_num, state) in list_of_states:
                line["state_num"] = problem_num
                line["state"] = str(state)
                f.write(json.dumps(line))
                f.write("\n")
 
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('json_file', type=str)
    parser.add_argument('--output_file', type=str, default="constrained_examples.jsonl")
    args = parser.parse_args()
    args.task_name = args.json_file.split("/")[-1].split(".")[0].lower()
    main(args)