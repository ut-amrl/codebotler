% Check every printer room for a stapler, and  come back and tell me which
%  ones do not have a stapler.

% go to every printer room
:- not t_go_to("printer room 1", _).
:- not t_go_to("printer room 2", _).
:- not t_go_to("printer room 3", _).

% don't go to a non-printer room
:- t_go_to(X, T), @contains_any(X, "printer", "start_loc") = 0.

% only check for stapler
:- t_is_in_room(X, _), X!="stapler".
 
% never say living room since it is not a printer room
:- t_say(X, _), @contains_all(X, "living") = 1.

% come back
:- not at("robot", "start_loc", timeout).

% trivially require actions
:- not t_go_to(_,_).
:- not t_say(_,_).
:- not t_is_in_room(_,_).

% say which printer room
say_printer_1 :- t_say(X, _), @contains_all(X, "stapler", "printer room 3", "not/no/don't/doesn't/n't") = 1.
say_printer_2 :- t_say(X, _), @contains_all(X, "stapler", "printer room 2", "not/no/don't/doesn't/n't") = 1.
say_printer_3 :- t_say(X, _), @contains_all(X, "stapler", "printer room 1", "not/no/don't/doesn't/n't") = 1.

#external state(0).
% stapler in printer room 1.
state_0.
:- not say_printer_2.
:- not say_printer_3.
:- say_printer_1.

#external state(1).
% stapler in living room.
state_1.
:- say_printer_1.
:- say_printer_2.
:- say_printer_3.

#external state(2).
% there is no stapler.
state_2.
:- say_printer_1.
:- say_printer_2.
:- say_printer_3.