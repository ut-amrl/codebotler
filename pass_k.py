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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path, help="Path to the JSON file.")
    args = parser.parse_args()

    df = pd.read_json(args.file, lines=True)
    evaluate_data(df, 1)

if __name__ == "__main__":
    main()