% Ask if Alice and Bob in their offices are up for lunch. If yes, tell them that 
% we'll meet in the lobby in 5 minutes. Come back and tell me who all are joining 
% for lunch.

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

meet(P) :- replied(P, "yes",T), t_say(X, T), 
        @contains_all(X, "meet", "lobby", "5 minutes/five minutes") = 1.

report_joining(P, T) :- replied(P, "yes", _), at("robot", "start_loc", T), t_say(X, T),
    @contains_all(X, P, "joining/participants/participate/join/come/coming", "lunch") = 1.

joining(P) :- report_joining(P, _).

:- option(X), @contains_any(X, "yes", "no") = 0.

#external state(0).
% both alice and bob join.
state_0.
:- not joining("bob").
:- not joining("alice").
:- not meet("alice").
:- not meet("bob").

#external state(1).
% neither alice or bob join.
state_1.
:- joining("bob").
:- joining("alice").
:- meet("alice").
:- meet("bob").

#external state(2).
% only alice joins.
state_2.
:- joining("bob").
:- not joining("alice").
:- not meet("alice").
:- meet("bob").

#external state(3).
% only bob joins.
state_3.
:- not joining("bob").
:- joining("alice").
:- meet("alice").
:- not meet("bob").