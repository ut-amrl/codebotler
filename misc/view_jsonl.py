import argparse
import json

def main(args):
    with open(args.eval_file, 'r') as f:
        for i,line in enumerate(f.readlines()):
            line = json.loads(line)
            print(f"==========LINE {i} in {args.eval_file}==========\n")
            print(line["name"])
            print(line["is_sat"])
            print(line["completion"])
            print("\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('eval_file', type=str)
    args = parser.parse_args()
    main(args)