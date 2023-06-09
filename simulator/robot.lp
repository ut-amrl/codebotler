
% ASP lightweight simulator for evaluating robot command traces
% INPUT: NL command and generated code
% eg.
%   "Go to Arjun's office, ask him if he is ready to head out, and come back and tell me what he said"
%    ```
%     start_loc = get_current_loc()
%     go_to("Arjun's office")
%     response = ask("Arjun", "ready to head out?", ["yes", "no", "maybe"])
%     go_to(start_loc)
%     say("Arjun said: " + response)
%   ```
% OUTPUT: whether generated code matches NL command
% METHOD: run a simulation of the code, check if it meets the NL command specifications


% Transition rules


% goto person = goto room where person is in
is_in_room("robot", B, T+1) :- is_in_room(Person, B, T), 
        not is_in_room(_, Person, 0), % person not a room
        is_in_room("robot", C, T), 
        go_to(Person, T), 
        T < timeout.

% goto changes loc of robot at next timestep
is_in_room("robot", B, T+1) :- go_to(B, T), 
                            not is_in_room(B, _, _), % is a room not person
                            is_in_room("robot", C, T), 
                            T < timeout.

% robot stays in same loc unless goto issued
is_in_room("robot", R, T+1) :- is_in_room("robot", R, T), 
                                not go_to(_, T), % goto R is no-op
                                T < timeout. 

% other agents stay in same loc (no control over other agents for now)
is_in_room(A, R, T+1) :- is_in_room(A, R, T), 
                        T < timeout, 
                        A!="robot".

% if agents are in the same room, ask question
asked(Agent, Question, Time) :- is_in_room(Agent, Room, Time), 
                            is_in_room("robot", Room, Time),
                            ask(Agent, Question, Time), 
                            Agent != "robot".


% if agent asked question, get response
replied(Agent, Answer, Time) :- asked(Agent, Question, Time),
                                r(Answer).


