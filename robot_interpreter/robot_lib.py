object_locations = {'stapler': "Yash's office"}
question_responses = iter([])
all_rooms = ['Kitchen', 'Living Room', 'Game Room', "Yash's office", "Arnold's office"]
current_location = "<starting room>"

def go_to(location):
    global current_location
    current_location = location
    print(f"Robot goes to {location}")


def get_current_location():
    return current_location


def object_in_room(object):
    object_location = object_locations.get(object, None)
    current_location = get_current_location()
    if object_location == current_location:
        print(f"{object} is in {current_location}")
        return True
    print(f"{object} is not in {current_location}")
    return False

def is_in_room(object):
    object_location = object_locations.get(object, None)
    return object_location == get_current_location()

def say(message):
    print(f"Robot says: \"{message}\"")

def get_all_rooms():
    global all_rooms
    return all_rooms

def ask(person, question, options=None):
    if options == None:
        print(f"Robot asks {person}: \"{question}\"")
    else:
        print(f"Robot asks {person}: \"{question}\" with options {options}")
    response = next(question_responses)
    print(f"Response: {response}")
    return response

def execute():
    start_loc = get_current_location()
    stapler_location = None
    for room in get_all_rooms():
        go_to(room)
        if is_in_room("stapler"):
            stapler_location = room
            break
    
    go_to(start_loc)
    if stapler_location:
        say("There is a stapler in the " + stapler_location)
    else:
        say("There are no staplers in the house.")