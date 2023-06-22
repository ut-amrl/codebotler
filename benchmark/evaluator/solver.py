from clingo import Control, ast
from clingo.solving import Model
from clingo.symbol import Function, String, Number
from typing import List 
import clingo
import random 
import re
from solve_utils import SolveUtils, model_to_str


    
class Context:

    def __init__(self, asp_rules_file="robot.lp", timeout=10, debug=True, debug_file="debug.lp"):
        self.debug = debug
        self.program_file = debug_file
        
        with open(debug_file, 'w') as f:
            f.write("#script (python)\n")
            # clear contents and add temporal functions
            f.write(open("solve_utils.py", 'r').read())
            f.write("#end.\n\n")

        self.ctl = Control()
        self.timeout=timeout
        # self.prg = Program()
        # self.ctl.register_observer(ProgramObserver(self.prg))
        self.ctl.load(asp_rules_file)
        self.add(f"#const timeout = {timeout}.")

        self.current_t = 0
        self.robot_location = "start_loc"
        self.all_simulation_rooms = []
        
          
    def add(self, atom, part = "base"):
        if self.debug and part == "base":
            with open(self.program_file, "a+") as f:
                f.write(atom +"\n")
    
        self.ctl.add(part, [], atom)   
        
    def add_constraints(self, constraints):
        if isinstance(constraints, str):
            self.add_str_constraints(constraints)
        elif isinstance(constraints, list):
            self.add_list_constraints(constraints)
            
    def add_str_constraints(self, constraints:str):
        """
        Constraints are divided into: init states
        and integrity constraints
        """
        world_states = []
        for constraint in constraints.split(".\n"):
            constraint = constraint.strip()
            
            ## HACKY for empty str
            if len(constraint) < 2:
                continue
            
            if not constraint.endswith("."):
                constraint += "."
                
            if ":-" in constraint:
                self.add(constraint)
            else:
                world_states.append(constraint)
                
        self.add_world_states(world_states)
    
    def add_list_constraints(self, constraints:List[str]):
        """
        Constraints are divided into: init states
        and integrity constraints
        """
        world_states = []
        
        for constraint in constraints:
            constraint.strip() 
            ## HACKY
            if len(constraint) < 2:
                continue
            
            if ":-" in constraint:
                self.add(constraint)
            else:
                world_states.append(constraint)
                
        self.add_world_states(world_states)
          
       
        
    def add_world_states(self, init_state=[]):
        normalized_init_state = []
        all_simulation_rooms = []
        for atom in init_state:
            # atom = atom.lower().replace("'", "")
            self.add(atom)
            normalized_init_state.append(atom)
            if "at" in atom or "go_to" in atom or "room" in atom:
                room = atom.split('"')[-2]
                # room = room.lower().replace("'", "")
                all_simulation_rooms.append(room)
        
        all_simulation_rooms = list(set(self.all_simulation_rooms + all_simulation_rooms))
        for room in all_simulation_rooms:
            self.add(f'room("{room}").')
            
        self.init_state = normalized_init_state
        self.all_simulation_rooms = sorted(all_simulation_rooms)        
        
        


    def ground_and_solve(self):
        self.ctl.ground(context=SolveUtils())
        self.ctl.solve()
        # self.ctl.solve(on_model=lambda m: print("\nAnswer: {}".format(m)))
        model = ""
        with self.ctl.solve(yield_=True) as hnd:
            for i,m in enumerate(hnd):
                # print(m)
                model = model_to_str(str(m))
                
            # TODO assume one model
            return (model, str(hnd.get()))
            
    def get_robot_location(self) -> str:
        # get robot location at max time (from is_in_room)
        return self.robot_location

    
    def get_simulation_rooms(self) -> List[str]:
        # get all "is in room" and "goto" values 
        return self.all_simulation_rooms
    
    
    def is_in_robot_location(self, obj : str) -> bool:
        # obj = obj.lower()
        # check if is in the room at curr time step
        curr_loc = self.get_robot_location()
        # record that a check was made
        self.add(f'check_at("{obj}", "{curr_loc}", {self.current_t}).')
        
        # curr_loc = curr_loc.lower().replace("'", "")
        # obj = obj.lower().replace("'", "")
        for atom in self.init_state:
            if f'at("{obj}", "{curr_loc}",' in atom:
                return True
            
        
        return False

        
    ## actions: ground after each action
    
    ## get most curr response
    def robot_say(self, message : str) -> None:
        # message = message.lower().replace("'", "")
        self.add(f't_say("{message}", {self.current_t}).')
        
    def robot_go_to(self, location : str) -> None:
        # location = location.lower()
        # issue goto
        # TODO: this is a problem when people = people location
        # location = location.lower().replace("'", "")
        if location not in self.all_simulation_rooms:
            self.all_simulation_rooms.append(location)
            self.all_simulation_rooms = sorted(self.all_simulation_rooms)
            self.add(f'room("{location}").')
        self.add(f't_go_to("{location}", {self.current_t}).')
        self.current_t += 1 
        self.robot_location = location

    def robot_ask(self, person : str, question : str, options: List[str]) -> str:
        for option in options:
            self.add(f'option("{option}").')
            
        option = random.sample(options, k=1)[0]
        
        # option = option.lower().replace("'", "")
        # person = person.lower().replace("'", "")
        # question = question.lower().replace("'", "")
        self.add(f't_ask("{person}", "{question}", {self.current_t}).')
        self.add(f'reply("{person}", "{option}", {self.current_t+1}).')
        self.current_t += 2
        return option
"""
Note:
- once grounded, you can't change that chunk of code
- adding to solver just adds dead branch to search (need custom propagator)
- custom observer is best for getting

"""     

def main():
   c = Context()
    
    
if __name__=="__main__":
    main()
    

