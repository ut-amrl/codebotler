"""
See completions.ipynb for an example of how to use this script.
"""
# Authors: Carolyn Jane Anderson, Joydeep Biswas, Arjun Guha, and Francesca Luchetti
#
# Copyright 2023 Northeastern University, Roblox, University of Texas at Austin, and
# Wellesley College.
#
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE
import pandas as pd
from pathlib import Path
from tqdm import tqdm
from typing import List, Union
import os
from misc.utils import write_dict_to_python_file, write_custom_to_python_file        
from models.OpenAIChatModel import OpenAIChatModel

def empty_completions(problems: pd.DataFrame):
    new_columns = problems.columns.tolist() + ["completion", "completion_settings"]
    return pd.DataFrame(columns=new_columns)

def read_completions_if_exists_python(completions_path: Path, problems: pd.DataFrame):
    return empty_completions(problems)

def build_worklist(
        problems: pd.DataFrame,
        num_completions: int,
        completions_path: str
    ):

    worklist = []
    for _, row in problems.iterrows():
        for j in range(len(row["prompts"])):
            for k in range(num_completions):
                save_fn = row["name"] + "_{}_{}.py".format(j, k)
                existed = os.path.exists(completions_path + "/" + save_fn)
                if existed:
                    # not generating duplicate completions
                    continue
                
                data = {
                    "prompt": row["prompts"][j],
                    "name": row["name"],
                    "save_fn": save_fn
                }
                worklist.append(data)

    return worklist

def completions(
    model,
    stop_sequences: List[str],
    temperature: float,
    top_p: float,
    max_tokens: int,
    prompt_prefix: str,
    prompt_suffix: str,
    problems: pd.DataFrame,
    completions_path: Union[Path, str],
    num_completions: int):

    os.makedirs(completions_path, exist_ok=True)
    completions_path = str(completions_path)

    worklist = build_worklist(problems, num_completions, completions_path)
    prompts = [prompt_prefix + item["prompt"] + prompt_suffix for item in worklist]
    prompts = prompts

    for index, completion in tqdm(
        enumerate(
            model.generate(prompts, stop_sequences, temperature, top_p, max_tokens)
        ),
        total=len(prompts),
        desc="Completions"
    ):
        item = worklist[index]

        save_fn = completions_path + "/" + item["save_fn"]

        name = item["name"]
        prompt = item["prompt"] 

        completion_settings = {
            "temperature": temperature,
            "top_p": top_p,
            "max_tokens": max_tokens,
            "prompt_prefix": prompt_prefix,
            "stop_sequences": stop_sequences,
        }
        if type(model) is not OpenAIChatModel:
            completion = prompt_suffix + completion
        elif not completion.startswith(prompt_suffix.strip()):
            completion = prompt_suffix + "\n" + completion

        write_dict_to_python_file("name", name, save_fn, write_mode="w")
        write_dict_to_python_file("prompt", prompt, save_fn, write_mode="a")
        write_dict_to_python_file("completion_settings", completion_settings, save_fn, write_mode="a")
        write_custom_to_python_file(f'completion = """{completion}\n"""', save_fn, write_mode="a")
