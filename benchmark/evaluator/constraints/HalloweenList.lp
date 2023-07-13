% Go to every office, and if there is anyone there, ask if they'd like a chocolate, 
% caramel, or gummy. Come back and tell me how many of each we need to buy.

:- not at("robot", "start_loc", timeout).

chocolate2 :- t_say(X, T), at("robot","start_loc",T), 
                @contains_all(X, "two chocolate/2 chocolate") = 1, 
                @contains_all(X, "zero gumm/0 gumm", "zero caramel/0 caramel") = 1.
                
chocolate1_gummy1 :- t_say(X, T),  at("robot","start_loc",T), 
                @contains_all(X, "zero caramel/0 caramel") = 1,
                @contains_all(X, "one chocolate/1 chocolate", "one gumm/1 gumm") = 1.

#external state(0).
% 2 chocolates
state_0.
:- not chocolate2.

#external state(1).
% 1 chocolate, 1 gummy
state_1.
:- not chocolate1_gummy1.