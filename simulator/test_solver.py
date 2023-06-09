from solver import Context

'''
Note: make sure timeout can cover all steps
'''

def test_time():
    c = Context()
    # NL command:
    c.ctl.add("init_states", [], 'is_in_room("robot", "start_loc", 0).')
    c.ctl.add("init_states", [], 'is_in_room("Arjun", "office", 0).')
    c.ctl.add("")
    # generated LLM code:
    loc1 = c.get_current_location()
    tp1 = c.curr_tp
    c.go_to("Arjun's office")
    loc2 = c.get_current_location()
    tp2 = c.curr_tp
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
  
    init_state = ['is_in_room("robot", "start_loc", 0).',
                    'is_in_room("Arjun", "office", 0).']
    c = Context(timeout=3, init_state=init_state)
    
    start_loc = c.get_current_location()
    c.go_to("office")
    response = c.ask("Arjun", "ready to head out?", ["yes", "no"])
    c.go_to(start_loc)
    c.say("Arjun said: " + response)
    
    c.ctl.add(f':- not is_in_room("robot", "start_loc", {c.timeout}).')
    c.ctl.add(f':- not replied("Arjun", _, _).')
    (model, is_success) = c.ground_and_solve()
    assert(str(is_success) == "SAT"), (model, str(is_success))
    

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
    init_state = ['is_in_room("robot", "start_loc", 0).',
                    'is_in_room("mug", "living_room", 0).']
    c = Context(timeout=3, init_state=init_state)
    start_loc = c.get_current_location()
    c.go_to("living_room")
    mug_found = c.is_in_room("mug")
    c.go_to(start_loc)
    if mug_found:
        c.say("There is a mug in the living room")
    else:
        c.say("There is no mug in the living room")
        
    c.ctl.add(':- not say("There is a mug in the living room", _).')
    c.ctl.add(f':- not is_in_room("robot", "start_loc", {c.timeout}).')
    (model, is_success) = c.ground_and_solve()
    assert(str(is_success) == "SAT"), (model, str(is_success))
    
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
    init_state = ['is_in_room("robot", "start_loc", 0).',
                    'is_in_room("stapler", "laundry", 0).',]
    c = Context(timeout=3, init_state=init_state)
    list_of_rooms = c.get_all_rooms()
    start_loc = c.get_current_location()
    stapler_found = False
    stapler_loc = None
    for room in list_of_rooms:
        if "printer" not in room:
            continue
        c.go_to(room)
        if c.is_in_room("stapler"):
            stapler_found = True
            stapler_loc = room
            break
    c.go_to(start_loc)
    if stapler_found:
        c.say("There is a stapler in the " + stapler_loc)
    else:
        c.say("There is no stapler in the house")
        
    c.ctl.add(':- not say("There is no stapler in the house", _).')
    c.ctl.add(f':- not is_in_room("robot", "start_loc", {c.timeout}).')
    (model, is_success) = c.ground_and_solve()
    assert(str(is_success) == "SAT"), (model, str(is_success))
    
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
    init_state = ['is_in_room("robot", "start_loc", 0).']
    c = Context(timeout=4, init_state=init_state)
    c.all_rooms.append("kitchen")
    
    list_of_rooms = c.get_all_rooms()
    start_loc = c.get_current_location()
    if "kitchen" in list_of_rooms:
        c.go_to("kitchen")
        c.say("Yash, make food!")
    else:
        c.say("Yash, J2 closes at 9:00 pm")
    c.go_to(start_loc)
        
    c.ctl.add(':- not say("Yash, make food!",_).')
    (model, is_success) = c.ground_and_solve()
    assert(str(is_success) ==  "SAT"), (model, str(is_success))
    
    
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
    init_state = ['is_in_room("robot", "start_loc", 0).', 
                  'is_in_room("Joe", "Joe\'s room", 0).',
                  'is_in_room("Jill", "Jill\'s room", 0).',
                  'is_in_room("Yash", "Yash\'s room", 0).'
    ]
    c = Context(timeout=5, init_state=init_state)
    c.all_rooms += ["Joe's room", "Jill's room", "Yash's room"]
    
    def get_number(person):
        start_loc = c.get_current_location()
        c.go_to(person)
        response = c.ask(person, "Choose number between 1 and 5", ["1","2","3","4","5"])
        c.go_to(start_loc)
        return response
    
    start_loc = c.get_current_location()
    joe_number = get_number("Joe")
    jill_number = get_number("Jill")
    sum_of_numbers = int(joe_number) + int(jill_number)
    
    c.go_to("Yash's room")
    c.say("The sum of Joe's number and Jill's number is " + str(sum_of_numbers))
    c.go_to(start_loc)
    
    c.ctl.add('''condition :- say("The sum of Joe\'s number and Jill\'s number is ''' + 
              str(sum_of_numbers) + '''", T),
              is_in_room("Yash", "Yash's room", T),
              is_in_room("robot", "Yash's room", T).''')
    c.ctl.add(':- not condition.')
    (model, is_success) = c.ground_and_solve()
    
    assert(str(is_success) ==  "SAT"), (model, str(is_success))
   
    
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
    init_state = ['is_in_room("robot", "start_loc", 0).']
    c = Context(timeout=4, init_state=init_state)
    c.all_rooms.append("living_room")
    
    list_of_rooms = c.get_all_rooms()
    start_loc = c.get_current_location()
    if "kitchen" in list_of_rooms:
        c.go_to("kitchen")
        c.say("Yash, make food!")
    else:
        c.say("Yash, J2 closes at 9:00 pm")
    c.go_to(start_loc)
        
    c.ctl.add(':- not say("Yash, make food!",_).')
    (model, is_success) = c.ground_and_solve()
    assert(str(is_success) ==  "UNSAT"), (model, str(is_success))
    
      
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
    init_state = ['is_in_room("robot", "start_loc", 0).', 'is_in_room("cat", "living_room", 0).']
    c = Context(timeout=2, init_state=init_state)
    c.all_rooms.append("living_room")
    c.ctl.configuration.solve.parallel_mode = 8
    c.ctl.configuration.solve.opt_mode = "ignore"
    start_loc = c.get_current_location()
    
    list_of_rooms = c.get_all_rooms()
    
    if c.is_in_room("cat"):
        ret = "The cat is in the " + start_loc
    else:  
        for room in list_of_rooms:
            c.go_to(room)
            if c.is_in_room("cat"):
                ret = "The cat is in the " + room
                break
            c.go_to(start_loc)
        
        
    c.ctl.add(':- not say("The cat is in the living_room", _).')
    (model, is_success) = c.ground_and_solve()
    assert(str(is_success) == "UNSAT"), (model, str(is_success))
    
def test_unsat_3():
    pass

def test_unsat_4():
    pass

def test_unsat_5():
    pass