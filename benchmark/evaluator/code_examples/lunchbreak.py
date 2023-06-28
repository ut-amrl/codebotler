import json 

program_good1 = """
# good
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_good2 = """
# good
alice_res = None
bob_res = None
go_to("alice's Office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("bob's Office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_good3 = """
# good
alice_res = None
bob_res = None
go_to("start_loc")
go_to("alice's Office")
if is_in_room("Alice"):
    alice_res = ask("alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("bob's Office")
if is_in_room("Bob"):
    bob_res = ask("bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("alice is joining for lunch")
if bob_res == "yes":
    say("bob is joining for lunch")
"""

program_good4 = """
# good
alice_res = None
bob_res = None
go_to("start_loc")
go_to("bob's Office")
if is_in_room("Bob"):
    bob_res = ask("bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("alice's Office")
if is_in_room("Alice"):
    alice_res = ask("alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("alice is joining for lunch")
if bob_res == "yes":
    say("bob is joining for lunch")
"""

program_good5 = """
# good: go to Alice office twice but consecutive
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Alice's office")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    go_to("Bob's office")
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_good6 = """
# good: go to Alice office twice but consecutive
all_rooms = get_all_rooms()
alice_res = None
bob_res = None
for room in all_rooms:
    if "Alice's office" in room:
        go_to("Alice's office")
        if is_in_room("Alice"):
            alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
            if alice_res == "yes":
                say("we will meet you in the lobby in 5 minutes")
    elif "Bob's office" in room:
        go_to("Bob's office")
        if is_in_room("Bob"):
            bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
            if bob_res == "yes":
                say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_good7 = """
# bad: return instead of say
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes","no","maybe"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes","no","maybe"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
msg = ""
if alice_res == "yes":
    msg += "Alice is joining for lunch, "
if bob_res == "yes":
    msg += "Bob is joining for lunch"
if msg != "":
    say(msg)
"""

program_bad1 = """
# bad: go to other ppl's office
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Charlie's office")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad2 = """
# bad: go to other ppl's office 2
alice_res = None
bob_res = None
go_to(Charlie's office)
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad3 = """
# bad: go to Alice office twice
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Alice's office")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad4 = """
# bad: go to start_loc in the middle
alice_res = None
bob_res = None
go_to("start_loc")
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad5 = """
# bad: did not check response for Alice
alice_res = None
bob_res = None
go_to("start_loc")
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad6 = """
# bad: did not check response for Bob
alice_res = None
bob_res = None
go_to("start_loc")
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad7 = """
# bad: did not go to Bob's office
alice_res = None
bob_res = None
go_to("start_loc")
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad8 = """
# bad: wrong option matches
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["ok", "no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["ok", "no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad9 = """
# bad: only one response option
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad10 = """
# bad: three response options
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes","no","maybe"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes","no","maybe"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad11 = """
# bad: did not handle check for Alice or Bob in room
go_to("Alice's office")
alice_res = ask("Alice", "are you up for lunch", ["yes","no"])
if alice_res == "yes":
    say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
bob_res = ask("Bob", "are you up for lunch", ["yes","no"])
if bob_res == "yes":
    say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

program_bad12 = """
# bad: return instead of say
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes","no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes","no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
msg = ""
if alice_res == "yes":
    msg += "Alice is joining for lunch, "
if bob_res == "yes":
    msg += "Bob is joining for lunch"
return msg
"""

program_bad13 = """
# bad: do not say at start_loc
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes","no"])
    if alice_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes","no"])
    if bob_res == "yes":
        say("we will meet you in the lobby in 5 minutes")
msg = ""
if alice_res == "yes":
    msg += "Alice is joining for lunch, "
if bob_res == "yes":
    msg += "Bob is joining for lunch"
say(msg)
go_to("start_loc")
"""

program_bad14 = """
# bad: do not say after response with yes
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes","no"])
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes","no"])
msg = ""
if alice_res == "yes":
    msg += "Alice is joining for lunch, "
if bob_res == "yes":
    msg += "Bob is joining for lunch"
say(msg)
go_to("start_loc")
"""

program_bad15 = """
# bad: wrong response handler
alice_res = None
bob_res = None
go_to("Alice's office")
if is_in_room("Alice"):
    alice_res = ask("Alice", "are you up for lunch", ["yes", "no"])
    if alice_res == "no":
        say("we will meet you in the lobby in 5 minutes")
go_to("Bob's office")
if is_in_room("Bob"):
    bob_res = ask("Bob", "are you up for lunch", ["yes", "no"])
    if bob_res == "no":
        say("we will meet you in the lobby in 5 minutes")
go_to("start_loc")
if alice_res == "yes":
    say("Alice is joining for lunch")
if bob_res == "yes":
    say("Bob is joining for lunch")
"""

with open("lunchbreak.json", "w") as f:
    data = [
        {"program": program_good1, "description": "good"},
        {"program": program_good2, "description": "good"},
        {"program": program_good3, "description": "good"},
        {"program": program_good4, "description": "good"},
        {"program": program_good5, "description": "good"},
        {"program": program_good6, "description": "good"},
        # {"program": program_good7, "description": "good"},
        {"program": program_bad1, "description": "bad: go to other ppl's office"},
        {"program": program_bad2, "description": "bad: go to other ppl's office 2"},
        {"program": program_bad3, "description": "bad: go to Alice office twice"},
        {"program": program_bad4, "description": "bad: go to start_loc in the middle"},
        {"program": program_bad5, "description": "bad: did not check response for Alice"},
        {"program": program_bad6, "description": "bad: did not check response for Bob"},
        {"program": program_bad7, "description": "bad: did not go to Bob's office"},
        {"program": program_bad8, "description": "bad: wrong option matches"},
        {"program": program_bad9, "description": "bad: only one response option"},
        {"program": program_bad10, "description": "bad: three response options"},
        {"program": program_bad11, "description": "bad: did not handle check for Alice or Bob in room"},
        {"program": program_bad12, "description": "bad: return instead of say"},
        {"program": program_bad13, "description": "bad: bad: do not say at start_loc"},
        {"program": program_bad14, "description": "bad: do not say after response with yes"},
        {"program": program_bad15, "description": "bad: wrong response handler"}
    ]
    json.dump(data, f)