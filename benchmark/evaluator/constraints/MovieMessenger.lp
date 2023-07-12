% Ask Sally in her office if she wants to go to the cinema with Mark. 
% Go to Mark's office and tell him Sally’s answer. If Sally says yes, ask Mark 
% whether he wants to leave at 4PM, 5PM, or 6PM - then go tell Sally what time 
% Mark is leaving.

:- not t_go_to("mark's office", 2).
% Robot interacts with Mark in step 1. I am not bothering to fully spell this out.

:- not t_go_to("sally's office", 5).
% Since we don't check that Mark says 5pm, we could in fact have a mind-reading
% robot. It is more likely that the robot has a successful interaction with Mark
% in Step 1.
:- t_say(X, 6), @contains_all(X, "5pm") = 0.

#external state(0).
% Sally says yes, Mark says 5pm.
state_0.