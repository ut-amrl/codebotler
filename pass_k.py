import argparse
import pandas as pd
import numpy as np
from pathlib import Path

def estimator(n: int, c: int, k: int) -> float:
    """
    Calculates 1 - comb(n - c, k) / comb(n, k).
    """
    if n - c < k:
        return 1.0
    return 1.0 - np.prod(1.0 - k / np.arange(n - c + 1, n + 1))

def evaluate_data(df: pd.DataFrame, k: int):
    df = df[["name", "constraint", "is_sat"]]
    df = df.groupby(["name", "constraint"])
    df = df.agg(n=("is_sat", pd.Series.count), 
                c=("is_sat", pd.Series.sum))
    df["pass1"]  = df.apply(lambda row: estimator(row["n"], row["c"], 1), axis=1)
    df = df.drop(columns=["n", "c"])
    print(df)

def evaluate_data_2(df: pd.DataFrame, k: int):
    df = df[["name", "constraint", "completion", "is_sat"]]
    df = df.groupby(["name", "completion"])
    # Count items in each group
    df = df.agg(all_sat=("is_sat", pd.Series.all))

    df = df.reset_index()
    df = df.groupby(["name"])
    df = df.agg(c=("all_sat", pd.Series.sum))
    # WARNING: 20 hard-coded below

    df["pass1"]  = df.apply(lambda row: estimator(20, row["c"], 1), axis=1)
    df = df.drop(columns=["c"])

    print(df)

def evaluate_data_2(df: pd.DataFrame, k: int):
    df = df[["name", "constraint", "completion", "is_sat"]]
    df = df.groupby(["name", "completion"])
    # Count items in each group
    df = df.agg(all_sat=("is_sat", pd.Series.all))

    df = df.reset_index()
    df = df.groupby(["name"])
    df = df.agg(c=("all_sat", pd.Series.sum))
    # WARNING: 20 hard-coded below

    df["pass1"]  = df.apply(lambda row: estimator(20, row["c"], 1), axis=1)
    df = df.drop(columns=["c"])

    print(df)

def evaluate_data_3(df: pd.DataFrame, k: int):
    # State is unhashable, so we convert it to a string
    df["state_str"] = df["state"].apply(lambda x: str(x))
    df = df[["name", "state_str", "constraint", "completion", "is_sat"]]
    
    df = df.groupby(["name", "state_str"])
    df = df.agg(c=("is_sat", pd.Series.sum),
                n=("is_sat", pd.Series.count))
    
    df["pass1"]  = df.apply(lambda row: estimator(row["n"], row["c"], 1), axis=1)
    df = df.drop(columns=["c"])

    print(df)

    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path, help="Path to the JSON file.")
    args = parser.parse_args()

    df = pd.read_json(args.file, lines=True)
    #evaluate_data(df, 1)
    evaluate_data_3(df, 1)

if __name__ == "__main__":
    main()