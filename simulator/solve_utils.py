from clingo.symbol import Number, Function
from clingo.ast import parse_string
import re

class SolveUtils:
    

    def get_first(self, actions):
        arg = str(actions)
        arg = arg[:arg.index("empty")].rsplit("(")[-1].split(",")[0]
        return Function(arg, [])

    def contains(self, atom, pattern):
        atom = str(atom)[1:-1]
        pattern = str(pattern)[1:-1] #remove quotes
        if pattern in atom:
            return Number(1)
        else:
            return Number(0)

    def contains_any(self, atom, *patterns):
        atom = str(atom)[1:-1]
        patterns = [str(p)[1:-1] for p in patterns] #remove quotes
        if any([p in atom for p in patterns]):
            return Number(1)
        else:
            return Number(0)

        
    def contains_all(self, atom, *patterns):
        """
        Supports patterns in format ["p0", "p1/p2/p3"] where
        atom must contain p0 AND any of p1, p2, p3
        """
        atom = str(atom)[1:-1]
        patterns = [str(p)[1:-1] for p in patterns] #remove quotes
        if all([check_in(p,atom) for p in patterns]):
            return Number(1)
        else:
            return Number(0)

def check_in(pattern, atom):
        if "/" in pattern:
            patterns = pattern.split("/")
            return any([p in atom for p in patterns])
        else:
            return pattern in atom
        
def model_to_str(model:str) -> str:
    """
    Print model ordered by timesteps
    HACKY
    """
    modelstr = ""
    temporal_atoms = []
    other_atoms = []
    
    
    for atom in model.split(')')[:-1]:#hacky
        if atom[-1].isdigit(): 
            temporal_atoms.append(atom.strip()+")")
        else:
            other_atoms.append(atom.strip()+")")
    
    ordered_model = sorted(temporal_atoms, key=lambda x:int(re.findall(r'\d+',x)[-1]))
    modelstr += (f"Answer:\n"+"\n".join(ordered_model + other_atoms) + "\n")
    return modelstr

def on_model(m):
    print(model_to_str(str(m)))

def main(prg):
    prg.ground([("base", [])], context=SolveUtils())
    prg.solve(on_model=on_model)
