import json 
"""
The format of json (first version)

{
    "prompt": str,
    "program": str,
    "setup_eval_instruction" : {
        "init_state" : [str],
        "timeout": int,
        "rooms": [str]
    }
    "wrap_up_eval_instruction": [
        { "condition": str, "variables" : [str] }
    ],
    "is_sat": bool
}

1. Later if we need to evaluate on some customized function callbacks,
this should be able to extend to incoporate that. 
2. One problem right now is we don't have variable name checking for the program
    - if a variable's name is slightly off from what we specified in the json it will fail
3. indentation for program is very important!!
"""

def dump_to_json(jsonl_file, program, constraints):
    dict_to_dump = {"prompt": program, "constraint": constraints}
    print(dict_to_dump)
    with open(jsonl_file, "a") as f:
        json.dump(dict_to_dump, f)
    
    

def main():
    program = """
start_loc = get_current_loc()
go_to("Arjun's office")
response = ask("Arjun", "ready to head out?", ["yes", "no", "maybe"])
go_to(start_loc)
say("Arjun said: " + response)"""

    constraints = """
is_in_room("robot", "start_loc", 0).
is_in_room("Arjun", "Arjun's office", 0).
:- not is_in_room("robot", "start_loc", timeout).
:- not replied("Arjun", _, _).
""" 
    
    dump_to_json("benchmark_generator/test_comp.jsonl", program, constraints)

if __name__=="__main__":
    main()

# prompt = """
# Go to Joe’s room and ask him for a number between 1 to 5. 
# Do the same for Jill. Come to Yash’s room and tell them the
# sum of the two chosen numbers. Everyone is in their respective rooms
# """

# program = """
# def get_number(person):
#     start_loc = get_current_location()
#     go_to(person)
#     response = ask(person, "Choose number between 1 and 5", ["1","2","3","4","5"])
#     go_to(start_loc)
#     return response

# start_loc = get_current_location()

# joe_number = get_number("Joe")

# jill_number = get_number("Jill")

# sum_of_numbers = int(joe_number) + int(jill_number)

# go_to("Yash's room")

# say("The sum of Joe's number and Jill's number is " + str(sum_of_numbers))

# go_to(start_loc)
# """

# test_sat_5 = {
#     "prompt": prompt,
#     "program": program,
#     "setup_eval_instruction" : {
#         "init_state" : [
#                   'is_in_room("robot", "start_loc", 0).', 
#                   'is_in_room("Joe", "Joe\'s room", 0).',
#                   'is_in_room("Jill", "Jill\'s room", 0).',
#                   'is_in_room("Yash", "Yash\'s room", 0).'],
#         "timeout" : 5,
#         "rooms": ["Joe's room", "Jill's room", "Yash's room"]
#     },
#     "wrap_up_eval_instruction": [
#         {
#             "condition": '''condition :- say("The sum of Joe\'s number and Jill\'s number is ''' + 
#               "{}" + '''", T),
#               is_in_room("Yash", "Yash's room", T),
#               is_in_room("robot", "Yash's room", T).''',
#             "variables": ["sum_of_numbers"] # need to deal with variable naming change later
#         }
#         ,
#         {
#             "condition": ':- not condition.',
#             "variables": []
#         }
#     ],
#     "is_sat": True
# }

# with open("examples/test_sat_5.json", "w") as f:
#     json.dump(test_sat_5, f)



# test_sat_4 = {
#     "prompt": "If I have a kitchen, tell Yash to make food. Otherwise, remind Yash that J2 closes at 9:00 pm.",
#     "program": """
# list_of_rooms = get_all_rooms()
# start_loc = get_current_location()
# if 'kitchen' in list_of_rooms:
#     go_to('kitchen')
#     say('Yash, make food!')
# else:
#     say('Yash, no food')
# go_to(start_loc)
#     """,
#     "setup_eval_instruction" : {
#         "init_state" : [],
#         "timeout" : 4,
#         "rooms": ["kitchen"]
#     },
#     "wrap_up_eval_instruction": [
#         {
#             "condition": ':- not say("Yash, make food!",_).',
#             "variables": []
#         }
#     ],
#     "is_sat": True
# }

# with open("examples/test_sat_4.json", "w") as f:
#     json.dump(test_sat_4, f)


# prompt = "Find which room the cat is in, and come back and tell me."
# program = """
# start_loc = get_current_location()
# list_of_rooms = get_all_rooms()

# if is_in_room("cat"):
#     return "The cat is in the " + start_loc
    
# for room in list_of_rooms:
#     go_to(room)
#     if is_in_room("cat"):
#         return "The cat is in the " + room
#     go_to(start_loc)
    
# return "The cat is not in the house"
# """
# test_unsat_2 = {
#     "prompt": prompt,
#     "program": program,
#     "setup_eval_instruction" : {
#         "init_state" : ['is_in_room("robot", "start_loc", 0).', 'is_in_room("cat", "living_room", 0).'],
#         "timeout" : 2,
#         "rooms": ["living_room"]
#     },
#     "wrap_up_eval_instruction": [
#         {
#             "condition": ':- not say("The cat is in the living_room", _).',
#             "variables": []
#         }
#     ],
#     "is_sat": False
# }

# with open("examples/test_unsat_2.json", "w") as f:
#     json.dump(test_unsat_2, f)