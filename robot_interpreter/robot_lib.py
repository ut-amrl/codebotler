object_locations = {}
question_responses = iter(['Yes'])
current_location = "<starting room>"
current_object_held = "<None>"

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


def say(message):
    print(f"Robot says: \"{message}\"")


def ask(person, question, options=None):
    if options == None:
        print(f"Robot asks {person}: \"{question}\"")
    else:
        print(f"Robot asks {person}: \"{question}\" with options {options}")
    response = next(question_responses)
    print(f"Response: {response}")
    return response


def pick_up(object):
    global current_object_held
    current_object_held = object
    print(f"Robot picks up {object}")


def put_down(object):
    global current_object_held
    object_locations[object] = get_current_location()
    current_object_held = "<None>"
    print(f"Robot puts down {object}")

def execute():
    go_to("living room")
    if object_in_room("remote"):
        pick_up("remote")
        go_to("your desk")
        put_down("remote")
    else:
        go_to("your desk")
        say("There's no remote in the living room.")
    
    response = ask("you", "Do you want a cookie?", ["Yes", "No"])
    if response == "Yes":
        go_to("kitchen")
        pick_up("cookie")
        go_to("your desk")
        put_down("cookie")