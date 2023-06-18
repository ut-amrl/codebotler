% Transition rules

% robot goto
at("robot", B, T+1) :- t_go_to(B, T), 
                at("robot", A, T), 
                room(B), % destination is not a person
                T < timeout.

% robot goto person 
at("robot", B, T+1) :- t_go_to(P, T), 
                at(P, B, T), % destination is a person
                at("robot", A, T), 
                P != "robot", 
                T < timeout.

% robot inertia
at("robot", R, T+1) :- at("robot", R, T), T<timeout, 
                % not t_go_to(  X, T), room(X).
                    0{t_go_to(X, T): room(X)}0. % inertia unless goto

% non robot inertia
at(A, R, T+1) :- at(A, R, T), A!="robot", T < timeout.

% ask
asked(A, Q, T+1) :- at(A, R, T), 
                at("robot", R, T),
                A != "robot",
                t_ask(A, Q, T),
                T < timeout.

% after question asked, there is a response
replied(A, R, T) :- asked(A, Q, T),
                    reply(A, R, T).

% sanity check: no two places at once
:- at("robot", A, T), at("robot", B, T), B!=A.
