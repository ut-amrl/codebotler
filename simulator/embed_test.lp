% # - only go to “printer room”
% # - Don’t care the sequence of “printer rooms” being visited
% # - Only check for stapler
% # - Only say things about stapler and room (contains stapler, room, and no)
% # - Say must be correct (contain)

% Test contains + only
% Note: set logic not list logic
at("robot", "start_loc", 0).
at("stapler", "printer room 1", 0).
t_go_to("printer room 1", 1).
t_go_to("printer room 2", 2).
% t_go_to("living room", 2).
% check_at("stapler", "start_loc", 0).
check_at("stapler", "printer room 1", 1).
% check_at("mouse", "printer room 1", 1).
t_go_to("start_loc", 2).
t_say("no stapler here", 0).
% t_say("no  here", 0).
% t_go_to("living room", 2).

% Check: only goes to rooms containing "printer"

% % count number of different possible values of has_printer
% num_unique_has_printer(S) :- S = #count{has_printer(@contains(X, "printer")) : t_go_to(X, T)}.

% % maximally populate has_printer with all possible values
% S{has_printer(@contains(X, "printer")) : t_go_to(X, T)}S :- num_unique_has_printer(S).

% % Assert there are no go_tos to rooms without "printer"
% :- has_printer(0).

:- t_go_to(X, T), @contains_any(X, "printer", "start_loc") = 0.

% Check: only check stapler in_room

:- check_at(X, _, _), X!="stapler".

% Check: only say things about stapler and room

:- t_say(X, _), @contains(X, "stapler") = 0.



% Test initially
% problem: cons works for sets but not for lists

% t_go_to("printer room", 1).
% t_go_to("conference room", 2).
% at("robot", "bedroom 2", 0).
% at("robot", "bedroom 6", 3).

% #const timeout = 3.
% #const n = 2.
% listLen(empty, 0).
% listLen(cons(X,Y, L), K + 1) :- at("robot",X,Y); listLen(L, K); K < n.
% is5List(L) :- listLen(L, 5).

% timesteps(empty, 0).
% #const n = timeout+1.
% timesteps(cons(X, Y, L), K + 1) :- at("robot",X,Y); timesteps(L, K); K < n, Y = K.
% timeline(L) :- listLen(L, n).

% f(@get_first(L)) :- isList(L).

% Test nth action

% Test finally


% #show left/1.
% #const timeout=2.
% at("robot", "bed", 0).
% at("robot", "table", 1).
% at("robot", "chair", timeout).
% hehe(1).

% % check robot goes sequentially: bed, table, chair
% 1{t(X, Y, Z): at("robot", "chair", Z), at("robot", "table", Y), at("robot", "bed", X)}1.
% % :- not t(1).
% after_cond :- t(X, Y, Z), X < Y, Y < Z.
% :- not after_cond.

% % for every statement at("robot", _, X) if no statement is after 
% % hehe(Y), will be p(1), otherwise p(0). Meaning any at("robot", _, X)
% % is before hehe(Y) 

% % %TODO maximize this chocie rule for correct answer
% % #count{p(@is_after(Y, X)) : at("robot", _, X), hehe(Y)}}. 
% % % at robot bed is first of all at robot
% % 2{p2(@is_after(Y, X)) : at("robot", _, X), at("robot", "chair", Y)}3. 

% % check certain action is first among all of its name
% % a(3 {p_at("robot", X, Y) : at("robot", X, Y)} 3).
% % NOTE: only works for "at" because "at" exists at every t-step
% listLen(empty, 0).
% #const n = timeout+1.
% listLen(cons(X, Y, L), K + 1) :- at("robot",X,Y); listLen(L, K); K < n, Y = K.
% isList(L) :- listLen(L, n).

% f(@get_first(L)) :- isList(L).

% #show at/3.
% #show isList/1.
% #show f/1.
% #show t/1.
% 1{f(@is_first(X)): at("robot", X, _)}1 :- at("robot", _, T).

% can be implemented straight asp
% def is_after(self, *args):
%         timesteps = [x.number for x in kwargs]
%         ordered = sorted(timesteps)
%         if ordered == timesteps:
%             return Number(1) #true
%         else:
%             return Number(0)  #false

#script (python)
from clingo.symbol import Number, Function
from clingo.ast import parse_string

class Temporal:
    

    def get_first(self, actions):
        arg = str(actions)
        arg = arg[:arg.index("empty")].rsplit("(")[-1].split(",")[0]
        return Function(arg, [])

    def contains(self, atom, pattern):
        atom = str(atom)[1:-1]
        pattern = str(pattern)[1:-1] #remove quotes
        if pattern in atom:
            return Number(1)
        else:
            return Number(0)

    def contains_any(self, atom, *patterns):
        atom = str(atom)[1:-1]
        patterns = [str(p)[1:-1] for p in patterns] #remove quotes
        if any([p in atom for p in patterns]):
            return Number(1)
        else:
            return Number(0)

    def contains_all(self, atom, *patterns):
        atom = str(atom)[1:-1]
        patterns = [str(p)[1:-1] for p in patterns] #remove quotes
        if all([p in atom for p in patterns]):
            return Number(1)
        else:
            return Number(0)


def main(prg):
    prg.ground([("base", [])], context=Temporal())
    prg.solve()

#end.
