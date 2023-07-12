% Go to the elevator. Wait until someone shows up and ask them if they are here 
% for the tour. If yes, welcome them to the university, ask them to follow you, 
% and take them to the main conference room. If not, wait for the next person. 
% When you get to the conference room, say you have arrived at the conference 
% room and also say enjoy your visit here!

:- not t_go_to("elevator", _).

robot_ask_tour :- at("robot", "elevator", T), at(P, "elevator", T), P != "robot",
    t_ask(P, X, _, T), @contains(X, "tour") = 1.
 
robot_welcome :- replied(_,"yes",T), t_say(X, T),
            @contains_all(X, "welcome", "university") = 1.

robot_follow :- replied(_,"yes",T), t_say(X, T),
            @contains_all(X, "follow/come") = 1, t_go_to("main conference room", T2),
            T <= T2.
   
robot_enjoy_visit :- at("robot", "main conference room", T), 
                t_say(X, T), @contains_all(X, "enjoy", "visit") = 1.

#external state(0).
% person is at elevator and says "yes".
state_0.
:- not robot_ask_tour.
:- not robot_follow.
:- not robot_welcome.
:- not robot_enjoy_visit.

#external state(1).
% person is at elevator and says "no".
state_1.
:- not robot_ask_tour.
:- robot_follow.
:- robot_welcome.
:- robot_enjoy_visit.
:- not at("robot", "elevator", timeout).

#external state(2).
% person is at start_loc.
state_2.
:- robot_ask_tour.
:- robot_follow.
:- robot_welcome.
:- robot_enjoy_visit.
:- not at("robot", "elevator", timeout).