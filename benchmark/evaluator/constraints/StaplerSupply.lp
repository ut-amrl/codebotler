% Check every printer room for a stapler, and  come back and tell me which
%  ones do not have a stapler.

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