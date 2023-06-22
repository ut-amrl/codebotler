import json 


stapler_good1 = """
# good
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""

stapler_good2 = """
# good
go_to("start_loc")
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""

stapler_good3 = """
# good
go_to("start_loc")
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = "there is no stapler in rooms: "
for room in rooms_not_have_stapler:
    message += room + " "
say(message)
"""

stapler_good4 = """
# good
go_to("start_loc")
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" not in room:
        continue
    go_to(room)
    if not is_in_room("stapler"):
        rooms_not_have_stapler.append(room)
go_to("start_loc")
message = "there is no stapler in rooms: "
for room in rooms_not_have_stapler:
    message += room + " "
say(message)
"""

stapler_bad1 = """
# bad: break the code once one is found
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
            break
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""

stapler_bad2 = """
# bad: not all rooms are iterated
all_rooms = get_all_rooms()
all_rooms = all_rooms[::2]
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""

stapler_bad3 = """
# bad: did not check for stapler
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("printer"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""

stapler_bad4 = """
# bad: did not go back to the start_loc
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""


stapler_bad5 = """
# bad: say the wrong thing
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
message = ""
go_to("start_loc")
for room in rooms_not_have_stapler:
    message += room + ", "
message += "have a stapler"
say(message)
"""

stapler_bad6 = """
# bad: say the wrong thing 2
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
message = ""
go_to("start_loc")
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a printer"
say(message)
"""


stapler_bad7 = """
# bad: go to all the rooms but not printer room
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    go_to(room)
    if not is_in_room("stapler"):
        rooms_not_have_stapler.append(room)
message = ""
go_to("start_loc")
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""


stapler_bad8 = """
# bad: hallucinate rooms
all_rooms = ["printer room 1", "whatever room", "printer room 2"]
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""


stapler_bad9 = """
# bad: syntax error
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_back_start_loc()
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""

stapler_bad10 = """
# bad: did not say but return message
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
return message
"""

stapler_bad11 = """
# bad: no go_to only say
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""


stapler_bad12 = """
# bad: interleave go_to start_loc with other printer rooms
go_to("start_loc")
all_rooms = get_all_rooms()[::-1]
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room:
        go_to(room)
        if not is_in_room("stapler"):
            rooms_not_have_stapler.append(room)
        else:
            go_to("start_loc")
go_to("start_loc")
message = ""
for room in rooms_not_have_stapler:
    message += room + ", "
message += "do not have a stapler"
say(message)
"""


stapler_bad13 = """
# bad: wrong is_in_room order
go_to("start_loc")
all_rooms = get_all_rooms()
rooms_not_have_stapler = []
for room in all_rooms:
    if "printer room" in room and not is_in_room("stapler"):
        go_to(room)
        rooms_not_have_stapler.append(room)
go_to("start_loc")
message = "there is no stapler in rooms: "
for room in rooms_not_have_stapler:
    message += room + " "
say(message)
"""

with open("staplerSupply.json", "w") as f:
    data = [
        {"program": stapler_good1, "description": "good"},
        {"program": stapler_good2, "description": "good"},
        {"program": stapler_good3, "description": "good"},
        {"program": stapler_good4, "description": "good"},
        {"program": stapler_bad1, "description": "bad: break the code once one is found"},
        {"program": stapler_bad2, "description": "bad: not all rooms are iterated"},
        {"program": stapler_bad3, "description": "bad: did not check for stapler"},
        {"program": stapler_bad4, "description": "bad: did not go back to the start_loc"},
        {"program": stapler_bad5, "description": "bad: say the wrong thing"},
        {"program": stapler_bad6, "description": "bad: say the wrong thing 2"},
        {"program": stapler_bad7, "description": "bad: go to all the rooms but not printer room"},
        {"program": stapler_bad8, "description": "bad: hallucinate rooms"},
        {"program": stapler_bad9, "description": "bad: syntax error"},
        {"program": stapler_bad10, "description": "bad: did not say but return message"},
        {"program": stapler_bad11, "description": "bad: no go_to only say"},
        {"program": stapler_bad12, "description": "bad: interleave go_to start_loc with other printer rooms"},
        {"program": stapler_bad13, "description": "bad: wrong is_in_room order"}
    ]
    json.dump(data, f)
