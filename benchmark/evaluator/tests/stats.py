import argparse
from typing import List
import json
import pandas as pd
import argparse

num_gens = 20
tasks_to_num_states = {"StaplerSupply":3, "LunchBreak":4, "ElevatorTour":2}
# eval_files = ["../evals/openai_eval.jsonl","../evals/palm_eval.jsonl","../evals/starcoder_eval.jsonl"]

def main(args):
    for file in args.eval_files:
        
        #raw score
        with open(file, 'r') as f:
            raw_score = {k:0 for k in tasks_to_num_states.keys()}
            
            data = []
            print("\n", file)
            for i,line in enumerate(f.readlines()):
                line = json.loads(line)
                data.append(line)
                if line["is_sat"]:
                    raw_score[line["name"]] += 1
                    
            raw_score = {k:f"{v}/{tasks_to_num_states[k] * num_gens}" for k,v in raw_score.items()}        
            print("raw_score:", *list(raw_score.items()), sep="\n")
            # print("\n")
            df = pd.DataFrame(data)
            per_task_scores = {k:0 for k in tasks_to_num_states.keys()}
            
            # for each task, calculate per task (tot states) score
            for df in df.groupby("name"):
                task_name = df[0]
                k = tasks_to_num_states[task_name]
                i = 0
                while i < df[1].shape[0]:
                    task_scores = df[1].iloc[list(range(i, i+k))]
                    if task_scores["is_sat"].all():
                        per_task_scores[task_name] += 1
                    i += k
                
            print("per task completion score:", *list(per_task_scores.items()), sep="\n")
            print("pass@1 score:", *list({k:v/num_gens for k,v in per_task_scores.items()}.items()), sep="\n")
        

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('eval_files', type=str, nargs='+')
    args = parser.parse_args()
    main(args)