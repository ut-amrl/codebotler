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

    "staplersupply" : {
    "locations": ["printer room 1", "living room", 'printer room 2', "printer room 3", "start_loc"], 
    "objects": [{ "label": "stapler", "location": "printer room 1" }], 
    "interactive_agents": [], 
    "robot_location": "start_loc",
    },
    "lunchbreak1" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["yes"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["yes"]}],
    "robot_location" : "start_loc"
    },
    "lunchbreak2" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["no"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["no"]}],
    "robot_location" : "start_loc"
    },
    "lunchbreak3" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["yes"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["no"]}],
    "robot_location" : "start_loc"
    },
    "lunchbreak4" : {
    "locations" : ["alices office", "bobs office", "start_loc"],
    "objects" : [],
    "interactive_agents" : [{"name" : "alice", "location" : "alices office", "answers" : ["no"]},
                            {"name" : "bob", "location" : "bobs office", "answers" : ["yes"]}],
    "robot_location" : "start_loc"
    },
}

task_to_constraints = {
    
    "staplersupply" : '''

:- not t_go_to("printer room 1", _).
:- not t_go_to("printer room 2", _).
:- not t_go_to("printer room 3", _).

:- t_is_in_room(X, _), X!="stapler".
 
:- t_say(X, _), @contains_all(X, "stapler", "printer room 2", "not/no/dont") = 0.

:- t_say(X, _), @contains_all(X, "stapler", "printer room 3", "not/no/dont") = 0.
:- t_say(X, _), @contains_all(X, "living", "not/no/dont") = 1.
:- not at("robot", "start_loc", timeout).

:- not t_go_to(_,_).
:- not t_say(_,_).
:- not t_is_in_room(_,_).
        ''',
        #TODO: fix lunchbreak constraints
    "lunchbreak" : '''
both_no :- t_ask("alice", A, O, T1),@contains(A, "lunch")=1,
                t_ask("bob", B, O, T3), @contains(B, "lunch")=1,
                replied("alice","no",T2), replied("bob","no",T4),
                T1 < T2, T3 < T4,
                not t_say(_, _).
                
alice_yes :- t_ask("alice", A, O, T1),@contains(A, "lunch")=1,
            replied("alice","yes",T2), at("robot", "alices office", T3),
            t_say(X, T3), @contains_all(X, "lobby", "5 minutes/five minutes", "meet") = 1,
            t_say(R, T4), @contains_all(R, "alice")=1,  at("robot", "start_loc", T4),
            T1 < T2 < T3 < T4.
            
bob_yes :- t_ask("bob", B, O, T4), @contains(B, "lunch")=1,
            replied("bob","yes",T5), at("robot", "bobs office", T6), 
            t_say(Y, T6), @contains_all(Y, "lobby", "5 minutes/five minutes", "meet") = 1,
            t_say(R, T7), @contains_all(R, "bob")=1,  at("robot", "start_loc", T7),
            T4 < T5 < T6 < T7.
            

bob_no :- not bob_yes.
alice_no :- not alice_yes.
:- alice_no, alice_yes.
:- bob_no, bob_yes.
:- not alice_no, not alice_yes.
:- not bob_no, not bob_yes.


:- not option("yes"). 
:- not option("no").
:- option(X), @contains_any(X, "yes", "no") = 0.
:- not t_is_in_room("alice", "alices office", _).
:- not t_is_in_room("bob", "bobs office", _).

at_least_one_yes :- bob_yes.
at_least_one_yes :- alice_yes.

:- both_no, at_least_one_yes.
:- not both_no, not at_least_one_yes.

:- t_go_to(X, T), @contains_any(X, "alice", "bob", "start_loc") = 0.

:- not t_ask("alice",_,_).
:- not t_ask("bob",_,_).
:- not replied(_,_,_).
:- not at("robot","start_loc", timeout).

num_say_meet_lobby(S) :- S = #count{X: t_say(X, T), @contains_all(X, "lobby", "5 minutes/five minutes", "meet")=1}.
:-  bob_no, num_say_meet_lobby(S), S > 1.
:-  alice_no, num_say_meet_lobby(S), S > 1.
:- bob_no, alice_no, num_say_meet_lobby(S), S > 0.
''',

#     "elevatortour" : '''
# at("robot", "start_loc", 0).
# at("person", "elevator", 4).
# room("conference_room").

# :- t_go_to(X, T), @contains_any(X, "elevator", "start_loc") = 0.

# robot_welcome :- at("robot", "elevator", T), at(P, "elevator", T), P != "robot",
#     t_say(X, T), @contains_all(X, "welcome", "university") = 1.
    
# :- not robot_welcome.
# :- not at("robot", "conference_room", timeout).

# robot_enjoy_visit :- at("robot", "conference_room", T), 
#                 t_say(X, T), @contains_all(X, "enjoy", "visit") = 1.
# :- not robot_enjoy_visit.
# '''
}


def main(args):
    examples = json.loads(open(args.json_file,"r").read())
    
    with open(args.output_file,"w") as f:
        for line in examples:
            # rename program to completions
            line["completion"] = line.pop("program")
            # add constraint according to problem
            line["constraint"] = task_to_constraints[args.task_name]
            
            list_of_states = [v for k,v in task_to_states.items() 
                              if args.task_name in k.lower()]
            
            for state in list_of_states:
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