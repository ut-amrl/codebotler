from solver import Context

'''
Note: make sure timeout can cover all steps
'''

def test_time():
    c = Context()
    # NL command:
    c.add_constraints(['at("robot", "start_loc", 0).',
                       'at("Arjun", "office", 0).'])
    c.ctl.add("")
    # generated LLM code:
    loc1 = c.get_robot_location()
    tp1 = c.current_t
    c.robot_go_to("Arjun's office")
    loc2 = c.get_robot_location()
    tp2 = c.current_t
    assert(loc1 == "start_loc"), loc1
    assert(loc2 == "Arjun's office"), loc2
    assert(tp1 == 0), tp1
    assert(tp2 == 1), tp2
    
def test_sat_1():
    """
    Go to Arjun's office, ask him if he is ready to head out, and come back and tell me what he said

    start_loc = get_current_loc()
    go_to("Arjun's office")
    response = ask("Arjun", "ready to head out?", ["yes", "no", "maybe"])
    go_to(start_loc)
    say("Arjun said: " + response)
    """
  
    
    c = Context(timeout=3)
    c.add_constraints(['at("robot", "start_loc", 0).',
                    'at("Arjun", "office", 0).',
                    ':- not at("robot", "start_loc", timeout).',
                    ':- not replied("Arjun", _, _).'])
    start_loc = c.get_robot_location()
    c.robot_go_to("office")
    response = c.robot_ask("Arjun", "ready to head out?", ["yes", "no"])
    c.robot_go_to(start_loc)
    c.robot_say("Arjun said: " + response)
    
    
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success == "SAT"), (model, is_success)
    

def test_sat_2():
    """
    Check if there are any mugs in the living room, and come back and tell me if there are any
    
    start_loc = get_current_loc()
    go_to("living_room")
    mug_found = is_in_room("mug")
    go_to(start_loc)
    if mug_found:
        say("There is a mug in the living room")
    else:
        say("There is no mug in the living room")
    """
    constraints = ['at("robot", "start_loc", 0).',
                    'at("mug", "living_room", 0).',
                    ':- not t_say("There is a mug in the living room", _).',
                    ':- not at("robot", "start_loc", timeout).']
    c = Context(timeout=3)
    c.add_constraints(constraints)
    
    start_loc = c.get_robot_location()
    c.robot_go_to("living_room")
    mug_found = c.is_in_robot_location("mug")
    c.robot_go_to(start_loc)
    if mug_found:
        c.robot_say("There is a mug in the living room")
    else:
        c.robot_say("There is no mug in the living room")
        
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success == "SAT"), (model, is_success)
    
def test_sat_3():
    """
    Check if there are any staplers in the printer room
    
    list_of_rooms = get_all_rooms()
    start_loc = get_current_loc()
    stapler_found = False
    stapler_loc = None
    for room in list_of_rooms:
        if "printer" not in room:
            continue
        go_to(room)
        if is_in_room("stapler"):
            stapler_found = True
            stapler_loc = room
            break
    go_to(start_loc)
    if stapler_found:
        say("There is a stapler in the " + stapler_loc)
    else:
        say("There is no stapler in the house")
    
    """
    constraints = ['at("robot", "start_loc", 0).',
                    'at("stapler", "laundry", 0).',
                    ':- not t_say("There is no stapler in the house", _).',
                    ':- not at("robot", "start_loc", timeout).']
    c = Context(timeout=5)
    c.add_constraints(constraints)
    list_of_rooms = c.get_simulation_rooms()
    start_loc = c.get_robot_location()
    stapler_found = False
    stapler_loc = None
    for room in list_of_rooms:
        if "printer" not in room:
            continue
        c.robot_go_to(room)
        if c.is_in_robot_location("stapler"):
            stapler_found = True
            stapler_loc = room
            break
    c.robot_go_to(start_loc)
    if stapler_found:
        c.robot_say("There is a stapler in the " + stapler_loc)
    else:
        c.robot_say("There is no stapler in the house")
        
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success == "SAT"), (model, is_success)
    
def test_sat_4():
    """
    If I have a kitchen, tell Yash to make food. Otherwise, remind Yash that J2 closes at 9:00 pm.
    
    list_of_rooms = get_all_rooms()
    start_loc = get_current_loc()
    if "kitchen" in list_of_rooms:
        go_to("kitchen")
        say("Yash, make food!")
    else:
        say("Yash, no food")
    go_to(start_loc)
    """
    constraints = ['at("robot", "start_loc", 0).',
                   ':- not t_say("Yash, make food!",_).',
                   ':- t_say("Yash, J2 closes at 9:00 pm",_).']
    c = Context(timeout=4)
    c.all_simulation_rooms.append("kitchen")
    c.add_constraints(constraints)
    
    list_of_rooms = c.get_simulation_rooms()
    start_loc = c.get_robot_location()
    if "kitchen" in list_of_rooms:
        c.robot_go_to("kitchen")
        c.robot_say("Yash, make food!")
    else:
        c.robot_say("Yash, J2 closes at 9:00 pm")
    c.robot_go_to(start_loc)
        
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success ==  "SAT"), (model, is_success)
    
    
def test_sat_5():
    """
    Go to Joe’s room and ask him for a number between 1 to 5. 
    Do the same for Jill. Come to Yash’s room and tell them the
    sum of the two chosen numbers. Everyone is in their respective rooms

    def get_number(person):
        start_loc = get_current_location()
        go_to(person)
        response = ask(person, "Choose number between 1 and 5", ["1","2","3","4","5"])
        go_to(start_loc)
        return response

    start_loc = get_current_location()

    joe_number = get_number("Joe")

    jill_number = get_number("Jill")

    sum_of_numbers = int(joe_number) + int(jill_number)

    go_to("Yash's room")

    say("The sum of Joe's number and Jill's number is " + str(sum_of_numbers))

    go_to(start_loc)
    """
    constraints = ['at("robot", "start_loc", 0).', 
                  'at("Joe", "Joe\'s room", 0).',
                  'at("Jill", "Jill\'s room", 0).',
                  'at("Yash", "Yash\'s room", 0).'
    ]
    c = Context(timeout=5)
    c.add_constraints(constraints)
    # c.all_simulation_rooms += ["Joe's room", "Jill's room", "Yash's room"]
    
    def get_number(person):
        start_loc = c.get_robot_location()
        c.robot_go_to(person)
        response = c.robot_ask(person, "Choose number between 1 and 5", ["1","2","3","4","5"])
        c.robot_go_to(start_loc)
        return response
    
    start_loc = c.get_robot_location()
    joe_number = get_number("Joe")
    jill_number = get_number("Jill")
    sum_of_numbers = int(joe_number) + int(jill_number)
    
    c.robot_go_to("Yash's room")
    c.robot_say("The sum of Joe's number and Jill's number is " + str(sum_of_numbers))
    c.robot_go_to(start_loc)
    
    c.add_constraints([f'''condition :- at("Yash", "Yash\'s room", T),
              at("robot", "Yash\'s room", T),
              t_say("The sum of Joe\'s number and Jill\'s number is {str(sum_of_numbers)}", T).''', 
              ":- condition."
              ])
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    '''
    TODO: removed this test case because we need to include person
    name in room(X) but not needed for benchmark
    '''
    # assert(is_success ==  "SAT"), (model, is_success)

def test_sat_6():
    """ ProgPrompt-like test: "I want pizza, cake, and party hats for an upcoming party, but I'm not sure if I have them. Tell me what items I have and what I'm missing. The chef knows about what food items we have, and you can check the garage for everything else."

    start_loc = get_current_location()

    # Check the kitchen for food items
    go_to("kitchen")
    response_pizza = ask("chef", "Do we have pizza?", ["Yes", "No"])
    response_cake = ask("chef", "Do we have cake?", ["Yes", "No"])

    # Check the garage for other items
    go_to("garage")
    have_party_hats = is_in_room("Party Hats")

    # Go back to the start location
    go_to(start_loc)

    # Print out what we have and what we don't
    if have_party_hats and response_pizza == "Yes" and response_cake == "Yes":
        say("We have everything we need for the party!")
    else:
        if response_pizza == "No":
            say("We don't have pizza.")
        if response_cake == "No":
            say("We don't have cake.")
        if not have_party_hats:
            say("We don't have party hats.")
    """

    constraints = ['at("robot", "start_loc", 0).',
                   'at("chef", "kitchen", 0).']
    c = Context(timeout=4)
    c.all_simulation_rooms.append("pantry")
    c.add_constraints(constraints)

    start_loc = c.get_robot_location()
    c.robot_go_to("kitchen")
    response_pizza = c.robot_ask("chef", "Do we have pizza?", ["Yes", "No"])
    response_cake = c.robot_ask("chef", "Do we have cake?", ["Yes", "No"])
    print("Pizza:", response_pizza, "Cake:", response_cake)
    c.robot_go_to("garage")
    have_party_hats = c.is_in_robot_location("party hats")
    c.robot_go_to(start_loc)

    said = []
    if have_party_hats and response_pizza == "Yes" and response_cake == "Yes":
        ans_str = "We have everything we need for the party!"
        said.append(ans_str)
        c.robot_say(ans_str)
    else:
        if response_pizza == "No":
            ans_str = "We don't have pizza."
            said.append(ans_str)
            c.robot_say(ans_str)
        if response_cake == "No":
            ans_str = "We don't have cake."
            said.append(ans_str)
            c.robot_say(ans_str)
        if not have_party_hats:
            ans_str = "We don't have party hats."
            said.append(ans_str)
            c.robot_say(ans_str)
    
    c.add_constraints([
        ':- not at("robot", "start_loc", timeout).'
        ':- not replied("chef", _, _).',
    ])
    c.add_constraints([
        f':- not t_say("{ans_str}", _).' for ans_str in said
    ])

    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success ==  "SAT"), (model, is_success)

def test_sat_7():
    ''' Example from paper: Language Models as Zero Shot Planners
    You some paper I want to throw away. Search every room in my house for a trash can. If there's also a person there, ask them to throw the paper away.

    list_of_rooms = get_all_rooms()
    start_loc = get_current_location()

    for room in list_of_rooms:
        go_to(room)
        if is_in_room("trash can"):
            if is_in_room("person"):
                response = ask("", "Can you throw away this paper?", ["Yes", "No"])
                if response == "Yes":
                    break
            else:
                break
    go_to(start_loc)
    '''
    constraints = ['at("robot", "start_loc", 0).',
                   'at("person", "kitchen", 0).',
                   'at("trash can", "kitchen", 0).']
    c = Context(timeout=4)
    c.add_constraints(constraints)
    c.all_simulation_rooms.append("living room")
    c.all_simulation_rooms.append("Yash's Room")
    c.all_simulation_rooms.append("office")

    list_of_rooms = c.all_simulation_rooms
    start_loc = c.get_robot_location()

    for room in list_of_rooms:
        c.robot_go_to(room)
        if c.is_in_robot_location("trash can"):
            if c.is_in_robot_location("person"):
                response = c.robot_ask("person", "Can you throw away this paper?", ["Yes"])
                if response == "Yes":
                    break
            else:
                break
    c.robot_go_to(start_loc)

    c.add_constraints([
        ':- not at("robot", "start_loc", timeout).',
        ':- not at("robot", "kitchen", _).'
        ':- not replied("person", _, _).',
    ])
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success ==  "SAT"), (model, is_success)

def test_unsat_1():
    """
    If I have a kitchen, tell Yash to make food. Otherwise, remind Yash that J2 closes at 9:00 pm.
    
    list_of_rooms = get_all_rooms()
    start_loc = get_current_loc()
    if "kitchen" in list_of_rooms:
        go_to("kitchen")
        say("Yash, make food!")
    else:
        say("Yash, no food")
    go_to(start_loc)
    """
    constraints = ['at("robot", "start_loc", 0).', 
                   ':- not t_say("Yash, make food!",_).',
                   ':- t_say("Yash, J2 closes at 9:00 pm",_).']
    c = Context(timeout=4)
    c.all_simulation_rooms.append("living_room")
    c.add_constraints(constraints)
    
    list_of_rooms = c.get_simulation_rooms()
    print("ROOMS", list_of_rooms)
    start_loc = c.get_robot_location()
    if "kitchen" in list_of_rooms:
        c.robot_go_to("kitchen")
        c.robot_say("Yash, make food!")
    else:
        c.robot_say("Yash, J2 closes at 9:00 pm")
    c.robot_go_to(start_loc)
        
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success ==  "UNSAT"), (model, is_success)
    
      
def test_unsat_2():
    """
    Find which room the cat is in, and come back and tell me.
    
    start_loc = get_current_loc()
    list_of_rooms = get_all_rooms()
    
    if is_in_room("cat"):
        return "The cat is in the " + start_loc
        
    for room in list_of_rooms:
        go_to(room)
        if is_in_room("cat"):
            return "The cat is in the " + room
        go_to(start_loc)
        
    return "The cat is not in the house"
    """
    contraints = ['at("robot", "start_loc", 0).', 'at("cat", "living_room", 0).',
                  ':- not t_say("The cat is in the living_room", _).']
    c = Context()
    c.all_simulation_rooms.append("living_room")
    c.add_constraints(contraints)
    c.ctl.configuration.solve.parallel_mode = 8
    c.ctl.configuration.solve.opt_mode = "ignore"
    start_loc = c.get_robot_location()
    
    list_of_rooms = c.get_simulation_rooms()
    
    if c.is_in_robot_location("cat"):
        ret = "The cat is in the " + start_loc
    else:  
        for room in list_of_rooms:
            c.robot_go_to(room)
            if c.is_in_robot_location("cat"):
                ret = "The cat is in the " + room
                break
            c.robot_go_to(start_loc)
        
        
    (model, is_success) = c.ground_and_solve()
    print(model, is_success)
    assert(is_success == "UNSAT"), (model, is_success)
    
def test_unsat_3():
    pass

def test_unsat_4():
    pass

def test_unsat_5():
    pass

def main():
    test_sat_1()
    test_sat_2()
    test_sat_3()
    test_sat_4()
    test_sat_5()
    test_sat_6()
    test_sat_7()
    test_unsat_1()
    test_unsat_2()
    # test_unsat_3()
    # test_unsat_4()
    # test_unsat_5()
    
if __name__ == '__main__':
    main()