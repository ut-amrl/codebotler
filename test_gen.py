from benchmark.evaluator.evaluate import run_simulation
from benchmark.evaluator.code_examples.read_and_dump import task_to_states, task_to_constraints
import argparse

def run_prog(args):
    task = args.task_name
    prog = open(args.task_program_file,"r").read()
    for i in range(len((task_to_states[task]))):
        state = task_to_states[task][i]
        additional_constraints = task_to_states[task][i]["additional_constraints"]
        constraints = task_to_constraints[task]
        (model, is_sat) = run_simulation(prog, state, constraints+"\n"+additional_constraints, timeout = 10, 
                robot_asp_logic="benchmark/evaluator/robot.lp",
                debug_file=f"mydebug/mydebug_{task}_{i}.lp")

        print(i, is_sat)
        # print(model)
    
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('task_program_file', type=str)
    parser.add_argument('task_name', type=str)
    args = parser.parse_args()
    run_prog(args)