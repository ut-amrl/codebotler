import argparse
import json
from clingo.symbol import *

"""
This script takes a json file of example good/bad completions in
the format of staplerSupply.json and adds a constraint according to the
task name. The output is a jsonl file of the same format with the added
constraints.
"""
"""
alice_no :- t_ask("alice", A, T1),@contains(A, "lunch")=1,
            replied("alice","no",T2), at("robot", "alices office", T3),
            t_say(X, T3),@contains_all(X, "lobby", "5 minutes/five minutes", "meet") = 0,
            t_say(R, T4), @contains_all(R, "alice")=0,  at("robot", "start_loc", T4),
            T1 < T2 < T3 < T4.

bob_no :- t_ask("bob", B, T4), @contains(B, "lunch")=1,
            replied("bob","no",T5), at("robot", "bobs office", T6), 
            t_say(Y, T6), @contains_all(Y, "lobby", "5 minutes/five minutes", "meet") = 0,
            t_say(R, T7), @contains_all(R, "bob")=0,  at("robot", "start_loc", T7),
            T4 < T5 < T6 < T7.
"""

task_to_constraints = {
    
# Check every printer room for a stapler, and come back and tell me which ones do not have a stapler.
# Constraints:
# - only go to "printer room"
# - Don\'t care the sequence of "printer rooms" being visited
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
        ':- t_say(X, _), @contains_all(X, "stapler", "printer room 2", "not/no/dont") = 0.',
        #Check: says something along lines of "no stapler in printer room 3" 
        ':- t_say(X, _), @contains_all(X, "stapler", "printer room 3", "not/no/dont") = 0.',
        # come back
        ':- not at("robot", "start_loc", timeout).'
        # trivial reqs
        ':- not t_go_to(_,_).',
        ':- not t_say(_,_).',
        ':- not check_at(_,_,_).',
        
    ],
    # Ask if Alice and Bob in their offices are up for lunch. If yes, 
    # tell them that we'll meet in the lobby in 5 minutes. Come back 
    #  and tell me who all are joining for lunch.
    "lunchbreak" : '''
at("robot", "start_loc",0).
at("alice", "alices office", 0).
at("bob", "bobs office", 0).


both_no :- t_ask("alice", A, T1),@contains(A, "lunch")=1,
                t_ask("bob", B, T3), @contains(B, "lunch")=1,
                replied("alice","no",T2), replied("bob","no",T4),
                T1 < T2, T3 < T4,
                not t_say(_, _).
                
alice_yes :- t_ask("alice", A, T1),@contains(A, "lunch")=1,
            replied("alice","yes",T2), at("robot", "alices office", T3),
            t_say(X, T3), @contains_all(X, "lobby", "5 minutes/five minutes", "meet") = 1,
            t_say(R, T4), @contains_all(R, "alice")=1,  at("robot", "start_loc", T4),
            T1 < T2 < T3 < T4.
            
bob_yes :- t_ask("bob", B, T4), @contains(B, "lunch")=1,
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
:- not check_at("alice", "alices office", _).
:- not check_at("bob", "bobs office", _).

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
'''
#     "lunchbreak" : [
# '''meet_lobby_cond :- t_ask("alice", A, T1),@contains(A, "lunch")=1,
#             t_ask("bob", B, T2), @contains(B, "lunch")=1,
#             replied(P,"yes",T3), t_say(X, T4), @contains_all(X, "lobby", "5 minutes/five minutes", "meet") = 1,
#             T1 < T2, T4=T3+1.''',

# '''not_meet_lobby_cond :- t_ask("alice", A, T1),@contains(A, "lunch")=1,
#                 t_ask("bob", B, T2), @contains(B, "lunch")=1,
#                 replied("alice","no",_), replied("bob","no",_),
#                 not t_say(_, _), 
#                 T1 < T2.''',

# ':- not_meet_lobby_cond, meet_lobby_cond.',
# ':- not meet_lobby_cond, not not_meet_lobby_cond.',


# ':- not t_ask(_,_,_).',
# ':- not replied(_,_,_).',
# ':- not at("robot","start_loc", timeout).',

# '''reports_alice :- replied("alice", "yes", T1), t_say(X, T2),
#                     @contains_all(X, "alice")=1, T1 < T2.''',
                    
# '''reports_bob :- replied("bob", "yes", T1), t_say(X, T2),
#                     @contains_all(X, "bob")=1, T1 < T2.''', 

# ':- replied("bob", "yes",_), not reports_bob.',
# ':- replied("alice", "yes",_), not reports_alice.']
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