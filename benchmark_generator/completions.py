"""
This is an adaptation of the following program:

https://github.com/Wellesley-EASEL-lab/StudentEval/blob/main/completions.py


python3 completions.py \
    --model-type openai \
    --completions completions_gpt35turbo-0301.jsonl \
    --engine gpt-35-turbo-0301 \
    --api-base https://charlie-east.openai.azure.com/ \
    --api-version 2022-12-01 \
    --max-workers 20 \
"""


import pandas as pd
from pathlib import Path
import json
from concurrent.futures import ThreadPoolExecutor
from tqdm import tqdm
from text_generation import Client
import openai
import time
import os
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM

DEFAULT_STOP_SEQUENCES = [ "\ndef", "\nclass", "\nif", "\nprint"  ]

# The settings below are what we used for the Charlie paper.
TEMPERATURE = 0.2
TOP_P = 0.95
MAX_TOKENS = 256

# This is from MultiPL-E, written by Carolyn and Arjun.
def stop_at_stop_token(decoded_string, stop_tokens):
    """
    Produces the prefix of decoded_string that ends at the first occurrence of
    a stop_token.

    WARNING: the decoded_string *must not* include the prompt, which may have stop tokens
    itself.
    """
    min_stop_index = len(decoded_string)
    for stop_token in stop_tokens:
        stop_index = decoded_string.find(stop_token)
        if stop_index != -1 and stop_index < min_stop_index:
            min_stop_index = stop_index
    return decoded_string[:min_stop_index]

class OpenAIModel:

    def __init__(self, engine, api_base, api_version, api_key):
        self.engine = engine
        openai.api_type = "azure"
        openai.api_base = api_base
        openai.api_version = api_version
        openai.api_key = api_key

    def generate(self, prompts: list):
        for prompt in prompts:
            while True:
                try:
                    results = openai.Completion.create(
                        engine=self.engine,
                        prompt=prompt.rstrip(),
                        temperature=TEMPERATURE,
                        max_tokens=MAX_TOKENS,
                        top_p=TOP_P,
                        n=1,
                        stop=DEFAULT_STOP_SEQUENCES,
                    )
                    time.sleep(0.01)
                    break
                except openai.error.RateLimitError:
                    print("Rate limited...")
                    time.sleep(5)        
            completion = results["choices"][0]["text"]
            yield completion


class TextGenerationModel:

    def __init__(self, url, max_workers):
        self.client = Client(url, timeout=60)
        self.max_workers = max_workers

    def generate_one(
        self,
        prompt: str,
        max_new_tokens=MAX_TOKENS,
        stop_sequences=DEFAULT_STOP_SEQUENCES):
        text = ""
        for response in self.client.generate_stream(prompt,
            max_new_tokens=max_new_tokens,
            temperature=TEMPERATURE,
            top_p=TOP_P,
            stop_sequences=stop_sequences):
            if not response.token.special:
                text += response.token.text
        return stop_at_stop_token(text, stop_sequences)

    def generate(self, prompts: list):
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            for completion in executor.map(lambda key: self.generate_one(key.rstrip()), prompts):
                yield completion

class AutoModel:

    def __init__(self, batch_size, path):
        self.batch_size = batch_size
        self.model = AutoModelForCausalLM.from_pretrained(path, trust_remote_code=True, torch_dtype=torch.bfloat16).cuda()
        self.tokenizer = AutoTokenizer.from_pretrained(path, trust_remote_code=True, padding_side="left")
        if ("starchat" in path) or ("starcoder" in path) or ("santacoder" in path):
            self.tokenizer.pad_token = self.tokenizer.eos_token


    def generate_batch(self, prompts):
        encoded_prompts = self.tokenizer([p.rstrip() for p in prompts], padding=True, return_attention_mask=True, return_tensors='pt').to(0)
        max_input_tokens = encoded_prompts["input_ids"].shape[1]
        outputs = self.model.generate(
            **encoded_prompts, 
            do_sample=True,
            top_p=TOP_P,
            temperature=TEMPERATURE,
            max_length = MAX_TOKENS + max_input_tokens)
        decoded_outputs = self.tokenizer.batch_decode(outputs[:, max_input_tokens:], skip_special_tokens=True, clean_up_tokenization_spaces=False)
        return [ stop_at_stop_token(s, DEFAULT_STOP_SEQUENCES) for s in  decoded_outputs ]

    
    def generate(self, prompts: list):
        for i in range(0, len(prompts), self.batch_size):
            batch = prompts[i:i+self.batch_size]
            for completion in self.generate_batch(batch):
                yield completion

# completions_path is a .jsonl file with rows:
#     { "prompt": string, "completion": string, "problem": string }
# Read it into a dictionary keyed by (prompt, problem)
def read_completions_if_exists(completions_path: Path, interactions: pd.DataFrame):
    if not completions_path.exists():
        return { }

    completions = { }
    num_completions = 0
    with open(completions_path, "r") as f:
        for line in f:
            num_completions += 1
            item = json.loads(line)
            prompt = item["prompt"]
            problem = item["problem"]
            constraint = item["constraint"]
            # Check if the prompt/problem is not in the interactions data frame
            if not ((interactions["prompt"] == prompt) & (interactions["problem"] == problem)).any():
                continue
            
            completion = item["completion"]
            key = (prompt, problem)
            if key in completions:
                completions[key]["completions"].append(completion)
                assert(completions[key]["constraint"] == constraint)
            else:
                completions[key] = {
                    "prompt": prompt,
                    "completions": [ completion ],
                    "problem": problem,
                    "constraint": constraint
                }
    print(f"Found {num_completions} existing completions.")
    return completions

def build_worklist(prompts: pd.DataFrame, completions: dict, num_completions: int):
    worklist = [ ]
    interaction_keys = set()
    for _, row in prompts.iterrows():
        prompt = row["prompt"]
        problem = row["problem"]
        constraint = row["constraint"]
        key = (prompt, problem)
        interaction_keys.add(key)
        if key not in completions:
            completions[key] = {
                "prompt": prompt,
                "completions": [],
                "problem": problem,
                "constraint": constraint
            }
        else:
            pass

        num_existing = len(completions[key]["completions"])
        remaining = num_completions - num_existing
        if remaining < 1:
            continue
        worklist.extend([key] * remaining)
    completion_keys = set(completions.keys())
    print(f"Found {len(interaction_keys)} interaction keys and {len(completion_keys)} completion keys.")
    print(f"Built worklist of {len(worklist)} prompts.")
    return worklist

def generate_completions(
    model,
    prompt_prefix: str,
    problems_path: Path,
    completions_path: Path,
    num_completions: int):
    
    problems = pd.read_csv(problems_path)

    # prefix the prompt with the prompt prefix in every row of problems
    problems["prompt"] = prompt_prefix + problems["prompt"]
    print(f"Found {len(problems)} problems.")

    completions = read_completions_if_exists(completions_path, problems)

    worklist = build_worklist(problems, completions, num_completions)

    # Build the worklist of prompts to run and write existing prompts to the output file
    with completions_path.open("a") as f:

        for index, completion in tqdm(enumerate(model.generate([key[0] for key in worklist])), total=len(worklist)):
            key = worklist[index]
            f.write(json.dumps({
                    "prompt": key[0],
                    "problem": key[1],
                    "completion": completion,
                    "constraint": completions[key]["constraint"]
                }) + "\n")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--problems", type=Path, default=Path("problems.csv"))
    parser.add_argument("--completions", type=Path, required=True)
    parser.add_argument("--model-type", choices=["hf-textgen", "openai", "automodel"])
    parser.add_argument("--prompt-prefix", type=Path, default=Path("../code_generator/gpt_prompt.md"))
    parser.add_argument("--num-completions", type=int, default=20)
    parser.add_argument("--max-workers", type=int, required=True)

    # For a HuggingFace model
    parser.add_argument("--url", type=str)

    # For an OpenAI model
    parser.add_argument("--engine", type=str)
    parser.add_argument("--api-version", type=str)
    parser.add_argument("--api-base", type=str)

    # For a local model
    parser.add_argument("--model-path", type=str)
    args = parser.parse_args()

    if args.model_type == "hf-textgen":
        model = TextGenerationModel(args.url, args.max_workers)
    elif args.model_type == "openai":
        model = OpenAIModel(args.engine, args.api_base, args.api_version, os.getenv("OPENAI_API_KEY"))
    elif args.model_type == "automodel":
        model = AutoModel(args.max_workers, args.model_path)

    generate_completions(
        model,
        args.prompt_prefix.read_text(),
        args.problems,
        args.completions,
        args.num_completions)
    
if __name__ == "__main__":
    main()
