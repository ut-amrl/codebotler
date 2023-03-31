with open("robot_lib_template.txt", 'r') as f:
    template = f.read()

with open("code.txt", 'r') as f:
    code = f.read()  # This can be replaced with a call to the model to generate the code

template += "\n" + "\n".join(["    " + i for i in code.split("\n")])

object_locations = {}
responses = ["Yes"]

with open("robot_lib.py", 'w') as f:
    f.write(f"object_locations = {str(object_locations)}\n")
    f.write(f"question_responses = iter({str(responses)})\n")
    f.write(template)

import robot_lib

robot_lib.execute()
