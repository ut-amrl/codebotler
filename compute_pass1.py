import argparse
import os 
import pandas as pd 

def compute_pass1(args):
    fn_path = os.path.join(args.dir, "error_breakdown", "result.csv")
    df = pd.read_csv(fn_path)
    success_count = df.loc[df["name"] == "Success", "error_count"].values[0]
    total_count = df["error_count"].sum()
    print(f"Pass@1: {success_count / total_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default='eval_results')

    args = parser.parse_args()
    compute_pass1(args) 