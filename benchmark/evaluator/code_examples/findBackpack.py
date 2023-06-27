import json 

""" Prompt: 
"""


program_good1 = """
# good: will find the backpack, but inefficient 
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""


program_good2 = """
# good
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
            break

if not found_backpack:                
    for room in all_rooms:
        if ("office" in room):
                go_to(room)
            if is_in_room("black backpack"):
                while True:
                    while not is_in_room("person"):
                        sleep(1)
                    answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                    if answer == "yes":
                        found_backpack = True
                        break
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_good3 = """
# good
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    go_to(start_loc)
                    found_backpack = True
                    break
            break

if not found_backpack:                
    for room in all_rooms:
        if ("office" in room):
                go_to(room)
            if is_in_room("black backpack"):
                while True:
                    while not is_in_room("person"):
                        sleep(1)
                    answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                    if answer == "yes":
                        found_backpack = True
                        go_to(start_loc)
                        break
                break

if not found_backpack:
    go_to(start_loc)
    say("I did not find the backpack.")

"""


program_bad1 = """
# bad: Only go to confrence rooms and office rooms
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    go_to(room)
    if is_in_room("black backpack"):
        while True:
            while not is_in_room("person"):
                sleep(1)
            answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == "yes":
                found_backpack = True
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad2 = """
# bad: All confrence rooms must be visited before all office rooms
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("office" in room):
        go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("confrence" in room):
        go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad3 = """
# bad: All the confrence rooms must be visited before visiting any offices
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room) | ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")

"""

program_bad4 = """
# bad: All offices must be visited unless backpack is found
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
        go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad5 = """
# bad: All confrence rooms must be visited unless backpack is found
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad6= """
# bad: All confrence rooms and offices must be visited unless backpack is found
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
if is_in_room("black backpack"):
    while True:
        while not is_in_room("person"):
            sleep(1)
        answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == "yes":
                found_backpack = True
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad7 = """
# bad: Only find the black backpack
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad8 = """
# bad: Must find backpack, not any other object
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("stapler"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("stapler"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad9 = """
# bad: Only ask if the person is in the room
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == "yes":
                found_backpack = True
                break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == "yes":
                found_backpack = True
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad10 = """
# bad: comes back without asking the person
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            found_backpack = True
            break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            found_backpack = True
            break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")

"""

program_bad11 = """
# bad: Must return to the start location
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad12 = """
# bad: must wait if there is no human
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == "yes":
                found_backpack = True
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad13 = """
#bad: does not ask the correct question
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")

"""

program_bad14 = """
#bad: options must contain both yes and no (yes)
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")

 """

program_bad14_2 = """
#bad: program must ask both yes and no (no)
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [no])
                if answer == "yes":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [no])
                if answer == "yes":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")

"""

program_bad15 = """
#bad: program must continue to wait if a person says no
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while not is_in_room("person"):
                sleep(1)
            answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == yes:
                found_backpack = True
                break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while not is_in_room("person"):
                sleep(1)
            answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
            if answer == "yes":
                found_backpack = True
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")

"""

program_bad16 = """
#bad: program does not take correct answer to ask
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                found_backpack = True
                break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                found_backpack = True
                break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad17 = """
#bad: missmatch between ask and check
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yep":
                    found_backpack = True
                    break
                
for room in all_rooms:
    if ("office" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yep":
                    found_backpack = True
                    break
go_to(start_loc)
if not found_backpack:
    say("I did not find the backpack.")
"""

program_bad18 = """
#bad: program must say if the backpack was not found
 start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
            break

if not found_backpack:                
    for room in all_rooms:
        if ("office" in room):
                go_to(room)
            if is_in_room("black backpack"):
                while True:
                    while not is_in_room("person"):
                        sleep(1)
                    answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                    if answer == "yes":
                        found_backpack = True
                        break
                break
go_to(start_loc)
"""

program_bad19 = """
#bad: program must say it did not find the backpack only if the backpack was not found
start_loc = get_current_location()
all_rooms = get_all_rooms()
found_backpack = False
for room in all_rooms:
    if ("confrence" in room):
            go_to(room)
        if is_in_room("black backpack"):
            while True:
                while not is_in_room("person"):
                    sleep(1)
                answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                if answer == "yes":
                    found_backpack = True
                    break
            break

if not found_backpack:                
    for room in all_rooms:
        if ("office" in room):
                go_to(room)
            if is_in_room("black backpack"):
                while True:
                    while not is_in_room("person"):
                        sleep(1)
                    answer = ask("person", "Please put the black backpack in my basket.", [yes, no])
                    if answer == "yes":
                        found_backpack = True
                        break
                break
go_to(start_loc)
say("I did not find the backpack.") 
"""






with open("findBackpack.json", "w") as f:
    data = [
        {"program": program_good1, "description": "good: will find the backpack, but inefficient "},
        {"program": program_good2, "description": "good"},
        {"program": program_good2, "description": "good"},
        {"program": program_bad1, "description": "bad: Only go to confrence rooms and office rooms"},
        {"program": program_bad2, "description": "bad: All confrence rooms must be visited before all office rooms"},
        {"program": program_bad3, "description": "bad: All the confrence rooms must be visited before visiting any offices"},
        {"program": program_bad4, "description": "bad: All offices must be visited unless backpack is found"},
        {"program": program_bad5, "description": "bad: All confrence rooms must be visited unless backpack is found"},
        {"program": program_bad6, "description": "bad: All confrence rooms and offices must be visited unless backpack is found"},
        {"program": program_bad7, "description": "bad: Only find the black backpack"},
        {"program": program_bad8, "description": "bad: Must find backpack, not any other object"},
        {"program": program_bad9, "description": "bad: Only ask if the person is in the room"},
        {"program": program_bad10, "description":"bad: Comes back without asking the person"},
        {"program": program_bad11, "description":"bad: Must return to the start location"},
        {"program": program_bad12, "description":"bad: Must wait if there is no human"},
        {"program": program_bad13, "description":"bad: Does not ask the correct question"},
        {"program": program_bad14, "description":'bad: Options must contain both yes and no (yes)'},
        {"program": program_bad14_2, "description":'bad:  options must contain both yes and no (no)'},
        {"program": program_bad15, "description":'bad: program must continue to wait if a person says no'},
        {"program": program_bad16, "description":'bad: program does not take correct answer to ask'},
        {"program": program_bad17, "description":'bad: missmatch between ask and check'},
        {"program": program_bad18, "description":'bad: program must say if the backpack was not found'},
        {"program": program_bad19, "description":'bad: program must say it did not find the backpack only if the backpack was not found'},

    ]
    json.dump(data, f)