import pandas as pd
from pathlib import Path
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path, help="Path to completions, e.g., ../benchmark/evaluator/starcoder_completions.jsonl")
    args = parser.parse_args()

    df = pd.read_json(args.file, lines=True)

    df = df[["name", "completion"]]
    df = df.groupby(["name"])
    df = df.agg(n=("completion", pd.Series.count))
    print(df)


if __name__ == "__main__":
    main()