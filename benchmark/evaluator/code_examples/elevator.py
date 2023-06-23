import json 

program_good1 = """
# good
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""


program_good2 = """
# good
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room. Enjoy your visit here!")
"""

program_good3 = """
# good
go_to("elevator")
while True:
    while not is_in_room("person"):
        time.sleep(1)
    response = ask("", "Are you here for the tour?", ["Yes", "No"])
    if response == "Yes":
        say("Welcome to the university!")
        go_to("conference room")
        say("I have arrived at the conference room. Enjoy your visit here!")
        break
"""

program_good3 = """
# good
go_to("elevator")
while True:
    while not is_in_room("person"):
        time.sleep(1)
    response = ask("", "Are you here for the tour?", ["Yes", "No"])
    if response == "Yes":
        say("Welcome to the university!")
        go_to("conference room")
        say("I have arrived at the conference room. Enjoy your visit here!")
        break
"""

program_bad1 = """
# bad: does not go to elevator
start_loc = get_current_location()
go_to("office")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""

program_bad2 = """
# bad: does not wait until there is a person
start_loc = get_current_location()
go_to("office")
if is_in_room("person"):
    response = ask("", "Are you here for the tour?", ["Yes", "No"])
    if response == "Yes":
        say("Welcome to the university!")
        go_to("conference room")
        say("I have arrived at the conference room.")
        say("Enjoy your visit here!")
"""

program_bad3 = """
# bad: first say statement is incorrect (missing)
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")


"""

program_bad4 = """
# bad: second say statment is incorrect (missing)
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("Enjoy your visit here!")
"""

program_bad5 = """
# bad: third say statemtnt is incorrect(missing)
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
"""

program_bad6= """
# bad: does not ask the person
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""

program_bad7 = """
# bad: answers to ask statement are not recorded
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        ask("", "Are you here for the tour?", ["yes", "no"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""

program_bad8 = """
# bad: missmatch between ask response and question check
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["yep", "no"])
        if response == "Yes":
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""

program_bad9 = """
# bad: take person to confrence room ONLY if response is yes
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response:
            say("Welcome to the university!")
            break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""

program_bad10 = """
# bad: answer yes does not break the infinite loop
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
            go_to("conference room")
            say("I have arrived at the conference room. Enjoy your visit here!")
    time.sleep(1)

"""

program_bad11 = """
# bad: answer "no" results in being taken to the confrence room
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        say("Welcome to the university!")
        break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""

program_bad12 = """
# bad: answer of "no" does not result in continuing to wait
start_loc = get_current_location()
go_to("elevator")
while True:
    if is_in_room("person"):
        response = ask("", "Are you here for the tour?", ["Yes", "No"])
        if response == "Yes":
            say("Welcome to the university!")
        break
    time.sleep(1)
go_to("conference room")
say("I have arrived at the conference room.")
say("Enjoy your visit here!")
"""






with open("elevator.json", "w") as f:
    data = [
        {"program": program_good1, "description": "good"},
        {"program": program_good2, "description": "good"},
        {"program": program_bad1, "description": "bad: does not go to elevator"},
        {"program": program_bad2, "description": "bad: does not wait until there is a person"},
        {"program": program_bad3, "description": "bad: first say statement is incorrect (missing)"},
        {"program": program_bad4, "description": "bad: second say statment is incorrect (missing)"},
        {"program": program_bad5, "description": "bad: third say statemtnt is incorrect(missing)"},
        {"program": program_bad6, "description": "bad: does not ask the person"},
        {"program": program_bad7, "description": "bad: answers to ask statement are not recorded"},
        {"program": program_bad8, "description": "bad: missmatch between ask response and question check"},
        {"program": program_bad9, "description": "bad: take person to confrence room ONLY if response is yes"},
        {"program": program_bad10, "description": "bad: answer yes does not break the infinite loop"},
        {"program": program_bad11, "description": "bad: answer no results in being taken to the confrence room"},
        {"program": program_bad12, "description": "bad:  answer of no does not result in continuing to wait"},
    ]
    json.dump(data, f)