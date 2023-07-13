% Go to every office, and if there is anyone there, ask if they'd like a chocolate, 
% caramel, or gummy. Come back and tell me how many of each we need to buy.

:- not at("robot", "start_loc", timeout).

chocolate2 :- t_say(X, T), @contains_all(X, "chocolates/chocolate", "2/two") = 1, 
                @contains_any(X, "gummy", "gummies", "caramel","caramels") = 0.
                
chocolate1_gummy1 :- t_say(X, T), @contains_any(X, "caramels", "caramel") = 0,
                @contains_all(X, "one chocolate/1 chocolate", "1 gummy/one gummy") = 1.

#external state(0).
% 2 chocolates
state_0.
:- not chocolate2.

#external state(1).
% 1 chocolate, 1 gummy
state_1.
:- not chocolate1_gummy1.