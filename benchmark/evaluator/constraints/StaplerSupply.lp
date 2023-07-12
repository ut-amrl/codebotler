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
say_printer_1_no_stapler :- t_say(X, _), @contains_all(X, "stapler", "printer room 1", "not/no/don't/doesn't/n't") = 1.
say_printer_2_no_stapler :- t_say(X, _), @contains_all(X, "stapler", "printer room 2", "not/no/don't/doesn't/n't") = 1.
say_printer_3_no_stapler :- t_say(X, _), @contains_all(X, "stapler", "printer room 3", "not/no/don't/doesn't/n't") = 1.

#external state(0).
% stapler in printer room 1. 
% Say printer rooms 2,3 don't have stapler.
% Don't say printer room 1 has stapler.
state_0.
:- not say_printer_2_no_stapler.
:- not say_printer_3_no_stapler.
:- say_printer_1_no_stapler.

#external state(1).
% stapler in living room. Say no printer rooms have stapler.
state_1.
:- not say_printer_1_no_stapler.
:- not say_printer_2_no_stapler.
:- not say_printer_3_no_stapler.

#external state(2).
% there is no stapler. Say no printer rooms have stapler.
state_2.
:- not say_printer_1_no_stapler.
:- not say_printer_2_no_stapler.
:- not say_printer_3_no_stapler.