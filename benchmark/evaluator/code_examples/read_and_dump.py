import argparse
import json
from clingo.symbol import *
"""
This script takes a jsonl file as input and dumps the constraints
to respective tasks
"""

# Ask Sally in her office if she wants to go to the cinema with Mark. Go to
# Mark’s office and tell him Sally’s answer. If Sally says yes, ask Mark
# whether he wants to leave at 4PM, 5PM, or 6PM - then go tell Sally what 
# time Mark is leaving.
#
# In this scenario, Sally says yes. Robot goes to Mark and mark says 5pm.
movie_messenger_state_0 = {
    "locations": [ "sally's office", "mark's office" ],
    "interactive_agents" : [
        # 1. Sally says "yes, I want to go to movie!"
        { "name" : "sally", "location" : "sally's office", "answers" : ["yes"]},
        # 2. Mark says 5pm
        { "name" : "mark", "location" : "mark's office", "answers" : ["5pm"]},
    ],
    "objects": [],
    "additional_constraints" : "",
    "robot_location": "sally office"
}

movie_messenger_constraints = """
:- not t_go_to("mark's office", 2).
% Robot interacts with Mark in step 1. I am not bothering to fully spell this out.
:- not t_go_to("sally's office", 5).
% Since we don't check that Mark says 5pm, we could in fact have a mind-reading
% robot. It is more likely that the robot has a successful interaction with Mark
% in Step 1.
:- t_say(X, 6), @contains_all(X, "5pm") = 0.
"""

task_to_states = {

    "StaplerSupply" : [{
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [{ "label": "stapler", "location": "printer room 1" }], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    "additional_constraints" : ""
    },
    {
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [{ "label": "stapler", "location": "living room" }], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    "additional_constraints" : ""
    },
    {
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    "additional_constraints" : ""
    }],
    "LunchBreak" : [{
    "locations" : ["alice's office", "bob's office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["yes"]},
                            {"name" : "bob", "location" : "bob's office", "answers" : ["yes"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- not joining(\"bob\").\n:- not joining(\"alice\").\n:- not meet(\"alice\").\n:- not meet(\"bob\")."
    },
    {
    "locations" : ["alice's office", "bob's office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["no"]},
                            {"name" : "bob", "location" : "bob's office", "answers" : ["no"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- joining(\"bob\").\n:- joining(\"alice\").\n:- meet(\"alice\").\n:- meet(\"bob\")."
    },
    {
    "locations" : ["alice's office", "bob's office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["yes"]},
                            {"name" : "bob", "location" : "bob's office", "answers" : ["no"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- joining(\"bob\").\n:- not joining(\"alice\").\n:- meet(\"alice\").\n:- not meet(\"bob\")."
    },
    {
    "locations" : ["alice's office", "bob's office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alice's office", "answers" : ["no"]},
                            {"name" : "bob", "location" : "bob's office", "answers" : ["yes"]}],
    "robot_location" : "start_loc",
    "additional_constraints" : ":- not joining(\"bob\").\n:- joining(\"alice\").\n:- not meet(\"alice\").\n:- meet(\"bob\")."
    }],
    "ElevatorTour" : [{
        "locations" : ["elevator", "main conference room", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "person", "location" : "elevator", "answers" : ["yes"]}, # diff way of writing same person
                                {"name" : "", "location" : "elevator", "answers" : ["yes"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : ":- not robot_ask_tour.\n:- not robot_follow.\n:- not robot_welcome.\n:- not robot_enjoy_visit."
    },
    {
        "locations" : ["elevator", "main conference room", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "person", "location" : "elevator", "answers" : ["no"]}, # diff way of writing same person
                                {"name" : "", "location" : "elevator", "answers" : ["no"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : ":- not robot_ask_tour.\n:- robot_follow.\n:- robot_welcome.\n:- robot_enjoy_visit."
    },
    {
        "locations" : ["elevator", "main conference room", "start_loc"],
        "objects" : [],
        "interactive_agents" : [{"name" : "person", "location" : "start_loc", "answers" : ["yes"]}, # diff way of writing same person
                                {"name" : "", "location" : "start_loc", "answers" : ["yes"]}],
        "robot_location" : "start_loc",
        "additional_constraints" : ":- robot_ask_tour.\n:- robot_follow.\n:- robot_welcome.\n:- robot_enjoy_visit.\n:- not at(\"robot\", \"elevator\", timeout)."
    },
    ],
    "MovieMessenger": [ movie_messenger_state_0 ],
}


task_to_constraints = {
    
    "StaplerSupply" : '''

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

# alternative:
# :- t_ask(P,_,_,T), at(P, X, T), not at("robot", X, T).
    "LunchBreak" : '''
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

:- t_go_to(X, T), @contains_any(X, "alice", "bob", "start_loc") = 0.


:- not t_ask("alice",_,_,_).
:- not t_ask("bob",_,_,_).
:- not replied(_,_,_).
:- not at("robot","start_loc", timeout).

meet(P) :- replied(P, "yes",T), t_say(X, T), @contains_all(X, "meet", "lobby", "5 minutes/five minutes") = 1.
report_joining(P, T) :- replied(P, "yes", _), t_say(X, T), @contains_all(X, P, "joining/join/come/coming","lunch") = 1.
joining(P) :- report_joining(P, _).
:- report_joining(P, T), not at("robot", "start_loc", T).

:- option(X), @contains_any(X, "yes", "no") = 0.
''',

# Go to the elevator. Wait until someone shows up and ask them if they are here 
# for the tour. If yes, welcome them to the university, ask them to follow you, 
# and take them to the main conference room. If not, wait for the next person. 
# When you get to the conference room, say you have arrived at the conference 
# room and also say enjoy your visit here!

# Constraint:
# Only go to “elevator” 
# Must wait until there is a person
# Must say the correct thing at the correct time
# Must welcome to university
# Must say that they have arrived at the conference room
# Must say to enjoy the visit
# Must correctly ask the person if they are here for the conference
# Ask statement must exist
# Must record the answer
# Must use the response correctly
# Must take person to conference room only if the answer is yes
# If the answer is no, must wait for the next person

    "ElevatorTour" : '''

:- not t_go_to("elevator", _).

robot_ask_tour :- at("robot", "elevator", T), at(P, "elevator", T), P != "robot",
    t_ask(P, X, _, T), @contains(X, "tour") = 1.
 
robot_welcome :- replied(_,"yes",T), t_say(X, T),
            @contains_all(X, "welcome", "university") = 1.

robot_follow :- replied(_,"yes",T), t_say(X, T),
            @contains_all(X, "follow") = 1, t_go_to("main conference room", T2),
            T <= T2.
   
robot_enjoy_visit :- at("robot", "main conference room", T), 
                t_say(X, T), @contains_all(X, "enjoy", "visit") = 1.

''',

    "MovieMessenger" : movie_messenger_constraints,
}

def constrain_jsonl(args):
    with open(args.jsonl_file,"r") as f:
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
            
def constrain_json(args):
    '''
    Old format
    '''
    # args.task_name = args.jsonl_file.split("/")[-1].split(".")[0]
    examples = json.loads(open(args.jsonl_file,"r").read())
    
    with open(args.output_file,"w") as f:
        for line in examples:
                # add constraints per state
                # only for target tests
                # list of {state, test}
                # rename program to completions
                line["completion"] = line.pop("program")
                # modify program
                completion = ("def task_program():\n" + 
                            "\n".join(["    "+l for l in line["completion"].split("\n")]))
                
                line["completion"] = completion
                # add constraint according to problem
                line["constraint"] = task_to_constraints[args.task_name]
            
                tests = []
                for state in task_to_states[args.task_name]:
                    state_mod = {k:v for k,v in state.items() if k != "additional_constraints"}
                    
                    test = {"state" : state_mod, 
                            "test" : task_to_constraints[args.task_name] + "\n" + state["additional_constraints"]} 
                    tests.append(test)
                    
                line["tests"] = tests
                line["name"] = args.task_name
                # dump
                f.write(json.dumps(line))
                f.write("\n")
 
def main(args):
    constrain_jsonl(args)
    ## Old format:
    # constrain_json(args)
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('jsonl_file', type=str)
    parser.add_argument('--output_file', type=str, default="constrained_examples.jsonl")
    # parser.add_argument('--task_name', type=str, default=None)
    args = parser.parse_args()
    main(args)