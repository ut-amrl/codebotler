from clingo import Control, ast
from clingo.solving import Model
from clingo.symbol import Function, String, Number
from typing import List 
import clingo
import random 

class Context:

    def __init__(self, robot_cmds_file, init_state = [], timeout=10):
        self.ctl = Control()
        self.timeout=timeout
        self.ctl.load(robot_cmds_file)
        self.ctl.add(f"#const timeout = {timeout}.")

        self.curr_tp = 0
        self.curr_robot_loc = "start_loc"
        self.all_rooms = []
        self.curr_reply = ""
        self.add_init_state(init_state)
        
        
    def add_constraints(self, constraints:str):
        """
        Constraints are divided into: init states
        and integrity constraints
        """
        init_state = []
        
        for constraint in constraints.split("."):
            constraint.strip() 
            ## HACKY
            if len(constraint) < 2:
                continue
            constraint += "."
            
            if ":-" in constraint:
                self.ctl.add(constraint)
            else:
                init_state.append(constraint)
                
        self.add_init_state(init_state)
                
        
    def add_init_state(self, init_state=[]):
        self.init_state = init_state
        for atom in init_state:
            self.ctl.add(atom)
            if "is_in_room" in atom:
                room = atom.split('"')[3]
                if room != "start_loc":
                    self.all_rooms.append(room)
        
    def inc_curr_tp(self):
        self.curr_tp += 1


    def ground_and_solve(self):
        self.ctl.ground()
        self.ctl.solve()
        # self.ctl.solve(on_model=lambda m: print("\nAnswer: {}".format(m)))
        return self.debug_model()

        
    def debug_model(self):
        model = []
        with self.ctl.solve(yield_=True) as hnd:
            for i,m in enumerate(hnd):
                # print(m)
                model.append(str(m))
                
            return (model, hnd.get())
            
    def get_current_location(self) -> str:
        # get robot location at max time (from is_in_room)
        return self.curr_robot_loc

    
    def get_all_rooms(self) -> List[str]:
        # get all "is in room" and "goto" values 
        return self.all_rooms
    
    
    def is_in_room(self, obj : str) -> bool:
        obj = obj.lower()
        # check if is in the room at curr time step
        curr_loc = self.get_current_location()
        for atom in self.init_state:
            if f'is_in_room("{obj}", "{curr_loc}",' in atom:
                return True
        return False

        
    ## actions: ground after each action
    
    ## get most curr response
    def say(self, message : str) -> None:
        # get most current reply
        self.ctl.add(f'say("{message}", {self.curr_tp}).')
        
    def go_to(self, location : str) -> None:
        location = location.lower()
        # issue goto
        self.ctl.add(f'go_to("{location}", {self.curr_tp}).')
        self.inc_curr_tp() 
        self.curr_robot_loc = location

    def ask(self, person : str, question : str, options: List[str]) -> str:
        person = person.lower()
        # issue ask and r() options, get reply at T+1
        opt = random.sample(options, k=1)[0]
        
        self.curr_reply = opt
        self.ctl.add(f"r({opt}).")
        self.ctl.add(f'ask("{person}", "{question}", {self.curr_tp}).')
        return self.curr_reply
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
    

