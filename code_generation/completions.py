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
import json
from concurrent.futures import ThreadPoolExecutor
from tqdm import tqdm
from text_generation import Client
from typing import List, Union
import time
import os
from transformers import AutoTokenizer, AutoModelForCausalLM

def load_model(args):
  if args.model_type == "openai" or args.model_type == "openai-chat":
    # If there exists a ".openai_api_key" file, use that as the API key.
    if os.path.exists(".openai_api_key"):
      with open(".openai_api_key", "r") as f:
        openai_api_key = f.read().strip()
    else:
      openai_api_key = os.getenv("OPENAI_API_KEY")
    assert len(openai_api_key) > 0, \
        "OpenAI API key not found. " + \
        "Either create a '.openai_api_key' file or " + \
        "set the OPENAI_API_KEY environment variable."
    if args.model_type == "openai":
      model = OpenAIModel(model=args.model_name, api_key = openai_api_key)
    elif args.model_type == "openai-chat":
      # model = OpenAIChatModel(model=args.model_name, api_key = openai_api_key)
      raise NotImplementedError("OpenAI Chat model not yet implemented.")
  elif args.model_type == "palm":
    # If there exists a ".palm_api_key" file, use that as the API key.
    if os.path.exists(".palm_api_key"):
      with open(".palm_api_key", "r") as f:
        palm_api_key = f.read().strip()
    else:
      palm_api_key = os.getenv("PALM_API_KEY")
    assert len(palm_api_key) > 0, \
        "PaLM API key not found. " + \
        "Either create a '.palm_api_key' file or " + \
        "set the PALM_API_KEY environment variable."
    model = PaLMModel(model=args.model_name, api_key = palm_api_key)
  elif args.model_type == "automodel":
    model = AutoModel(batch_size=1, path=args.model_name)
  elif args.model_type == "hf-textgen":
    model = TextGenerationModel(args.model_name, args.max_workers)
  else:
    raise ValueError(f"Unknown model type: {args.model_type}")
  return model

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

class PaLMModel:
    def __init__(self, model: str, api_key: str):
        import google.generativeai as palm
        self.palm = palm
        palm.configure(api_key=api_key)
        self.model = model

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        for prompt in prompts:
            completion = self.palm.generate_text(
                model=self.model,
                prompt=prompt,
                temperature=temperature,
                stop_sequences=stop_sequences,
                top_p=top_p,
                max_output_tokens=max_tokens,
            )
            # Preview is rate-limited to 30/minute, so we sleep for 2 seconds.
            # https://developers.generativeai.google/models/language#model_attributes
            time.sleep(2)
            yield completion.result

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        return self.palm.generate_text(
            model=self.model,
            prompt=prompt,
            temperature=temperature,
            stop_sequences=stop_sequences,
            top_p=top_p,
            max_output_tokens=max_tokens,
        ).result

class OpenAIModel:
    def __init__(
            self,
            use_azure: bool = False,
            engine: Union[str, None] = None,
            model: Union[str, None] = None,
            api_base: Union[str, None] = None,
            api_version: Union[str, None] = None,
            api_key: str = ""):
        import openai as openai
        self.openai = openai
        self.engine = None
        self.model = None
        if use_azure:
            self.openai.api_type = "azure"
        # Throw an error if the user has specified both an engine and a model.
        if engine is not None and model is not None:
            raise ValueError(
                "Please specify either an OpenAI engine or a model, but not both."
            )
        if engine is not None:
            print("Using OpenAI engine: " + engine)
            self.engine = engine
        elif model is not None:
            print("Using OpenAI model: " + model)
            self.model = model
        if api_base is not None:
            self.openai.api_base = api_base
        if api_version is not None:
            self.openai.api_version = api_version
        self.openai.api_key = api_key

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        for prompt in prompts:
            kwargs = {
                "prompt": prompt,
                "temperature": temperature,
                "max_tokens": max_tokens,
                "top_p": top_p,
                "stop": stop_sequences
            }
            if self.engine is not None:
                kwargs["engine"] = self.engine
            elif self.model is not None:
                kwargs["model"] = self.model
            while True:
                try:
                    results = self.openai.Completion.create(**kwargs)
                    break
                except self.openai.error.RateLimitError:
                    print("Rate limited...")
                    time.sleep(5)
            completion = results["choices"][0]["text"]
            yield completion

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        kwargs = {
            "prompt": prompt,
            "temperature": temperature,
            "max_tokens": max_tokens,
            "top_p": top_p,
            "stop": stop_sequences
        }
        if self.engine is not None:
            kwargs["engine"] = self.engine
        elif self.model is not None:
            kwargs["model"] = self.model
        return self.openai.Completion.create(**kwargs)["choices"][0]["text"]

class TextGenerationModel:
    def __init__(self, url, max_workers):
        self.client = Client(url, timeout=60)
        self.max_workers = max_workers

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        text = ""
        for response in self.client.generate_stream(
            prompt,
            do_sample=True,
            max_new_tokens=max_tokens,
            temperature=temperature,
            top_p=top_p,
            stop_sequences=stop_sequences,
        ):
            if not response.token.special:
                text += response.token.text
        return stop_at_stop_token(text, stop_sequences)

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            for completion in executor.map(
                lambda key: self.generate_one(
                    key.rstrip(), stop_sequences, temperature, top_p, max_tokens
                ),
                prompts,
            ):
                yield completion


class AutoModel:
    def __init__(self, batch_size, path):
        import torch

        self.batch_size = batch_size
        self.model = AutoModelForCausalLM.from_pretrained(
            path, trust_remote_code=True, torch_dtype=torch.bfloat16
        ).cuda()
        self.tokenizer = AutoTokenizer.from_pretrained(
            path, trust_remote_code=True, padding_side="left"
        )
        if ("starchat" in path) or ("starcoder" in path) or ("santacoder" in path) or ("xgen" in path):
            self.tokenizer.pad_token = self.tokenizer.eos_token

    def generate_batch(
        self,
        prompts,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        encoded_prompts = self.tokenizer(
            [p.rstrip() for p in prompts],
            padding=True,
            return_attention_mask=True,
            return_tensors="pt",
        ).to(0)
        max_input_tokens = encoded_prompts["input_ids"].shape[1]
        outputs = self.model.generate(
            **encoded_prompts,
            do_sample=True,
            top_p=top_p,
            temperature=temperature,
            max_length=max_tokens + max_input_tokens,
        )
        decoded_outputs = self.tokenizer.batch_decode(
            outputs[:, max_input_tokens:],
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )
        return [stop_at_stop_token(s, stop_sequences) for s in decoded_outputs]

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        for i in range(0, len(prompts), self.batch_size):
            batch = prompts[i : i + self.batch_size]
            for completion in self.generate_batch(
                batch, stop_sequences, temperature, top_p, max_tokens
            ):
                yield completion

    def generate_one(
        self,
        prompt,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        encoded_prompt = self.tokenizer.encode(
            prompt.rstrip(),
            padding=True,
            return_attention_mask=True,
            return_tensors="pt",
        ).to(0)
        num_input_tokens = encoded_prompt.shape[1]
        output = self.model.generate(
            encoded_prompt,
            do_sample=True,
            top_p=top_p,
            temperature=temperature,
            max_length=max_tokens + num_input_tokens
        )
        decoded_output = self.tokenizer.decode(
            output[0, num_input_tokens:],
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False
        )
        return stop_at_stop_token(decoded_output, stop_sequences)


def read_df(p: Path):
    if p.suffix == ".jsonl":
        return pd.read_json(p, lines=True)
    elif p.suffix == ".csv":
        return pd.read_csv(p)
    elif str(p) == "/dev/null":
        return pd.DataFrame()
    else:
        raise Exception(f"Unrecognized file extension: {p.suffix}")

def empty_completions(problems: pd.DataFrame):
    new_columns = problems.columns.tolist() + ["completion", "completion_settings"]
    return pd.DataFrame(columns=new_columns)

def read_completions_if_exists(completions_path: Path, problems: pd.DataFrame):
    if not completions_path.exists():
        return empty_completions(problems)

    completions = read_df(completions_path)
    if len(completions) == 0:
        return empty_completions(problems)

    # Only select those rows of completions where the "prompt" is in "problems"
    completions = completions[completions["prompt"].isin(problems["prompt"])]
    print(f"Found {len(completions)} existing completions.")
    return completions


def build_worklist(
    prompts: pd.DataFrame,
    prompt_prefix: str,
    completions: pd.DataFrame,
    num_completions: int):
    """
    Builds a worklist of prompts that need completions, using existing completions to avoid
    duplicating work from previous runs.
    """
    worklist = []
    for _, row in prompts.iterrows():
        prompt = row["prompt"]
        # Select all rows of completions where "prompt" is the same as the current prompt
        completions_for_prompt = completions[(completions["prompt"] == prompt) & (completions["completion_settings"] == prompt_prefix)]
        for _ in range(num_completions - len(completions_for_prompt)):
            worklist.append(row.to_dict())
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
    if isinstance(completions_path, str):
        completions_path = Path(completions_path)

    completions = read_completions_if_exists(completions_path, problems)
    worklist = build_worklist(problems, prompt_prefix, completions, num_completions)
    prompts = [prompt_prefix + item["prompt"] + prompt_suffix for item in worklist]

    # Build the worklist of prompts to run and write existing prompts to the output file
    with completions_path.open("a") as f:
        for index, completion in tqdm(
            enumerate(
                model.generate(prompts, stop_sequences, temperature, top_p, max_tokens)
            ),
            total=len(prompts),
            desc="Completions"
        ):
            item = worklist[index]
            item["completion_settings"] = {
                "temperature": temperature,
                "top_p": top_p,
                "max_tokens": max_tokens,
                "prompt_prefix": prompt_prefix,
                "stop_sequences": stop_sequences,
            }
            item["completion"] = prompt_suffix + completion
            f.write(json.dumps(item))
            f.write("\n")

    return pd.DataFrame(worklist)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--prompts", type=Path, required=True)
    parser.add_argument("--completions", type=Path, required=True)
    parser.add_argument("--model-type", choices=["hf-textgen", "openai", "palm", "automodel"])
    parser.add_argument("--prompt-prefix", type=Path)
    parser.add_argument("--prompt-suffix", type=Path)
    parser.add_argument("--num-completions", type=int, default=20)
    parser.add_argument("--max-tokens", type=int, default=256)
    parser.add_argument("--top-p", type=float, default=0.95)
    parser.add_argument("--temperature", type=float, default=0.2)

    # For an automodel
    parser.add_argument("--batch-size", type=int)
    # For an hf-textgen model
    parser.add_argument("--max-workers", type=int)
    # For an hf-textgen model
    parser.add_argument("--url", type=str)
    # For an openai model
    parser.add_argument("--openai-engine", type=str)
    # For an openai model
    parser.add_argument("--openai-model", type=str)
    # For an openai model
    parser.add_argument("--openai-api-version", type=str)
    # For an openai model
    parser.add_argument("--openai-api-base", type=str)
    # For an openai model
    parser.add_argument("--openai-azure", action="store_true", default=False)
    # For an automodel
    parser.add_argument("--model-path", type=str)
    # For a palm model
    parser.add_argument("--palm-model", type=str)
    args = parser.parse_args()

    # Check that flags are set correctly for each model type
    if args.model_type == "hf-textgen":
        assert args.batch_size is None
        assert args.max_workers is not None
        assert args.url is not None
        assert args.openai_engine is None
        assert args.openai_model is None
        assert args.openai_api_version is None
        assert args.openai_api_base is None
        assert not args.openai_azure
        assert args.model_path is None
    elif args.model_type == "openai":
        assert args.batch_size is None
        assert args.max_workers is None
        assert args.url is None
        assert args.openai_engine is not None or args.openai_model is not None
        assert args.model_path is None
    elif args.model_type == "palm":
        assert args.batch_size is None
        assert args.max_workers is None
        assert args.url is None
        assert args.palm_model is not None
        assert args.openai_model is None
        assert args.openai_api_version is None
        assert args.openai_api_base is None
        assert args.model_path is None
    elif args.model_type == "automodel":
        assert args.batch_size is not None
        assert args.max_workers is None
        assert args.url is None
        assert args.openai_engine is None
        assert args.openai_model is None
        assert args.openai_api_version is None
        assert args.openai_api_base is None
        assert not args.openai_azure
        assert args.model_path is not None

    if args.model_type == "hf-textgen":
        model = TextGenerationModel(args.url, args.max_workers)
    elif args.model_type == "openai":
        model = OpenAIModel(args.openai_azure,
                            args.openai_engine,
                            args.openai_model,
                            args.openai_api_base,
                            args.openai_api_version,
                            os.getenv("OPENAI_API_KEY"))
    elif args.model_type == "palm":
        model = PaLMModel(args.palm_model, os.getenv("PALM_API_KEY"))
    elif args.model_type == "automodel":
        model = AutoModel(args.batch_size, args.model_path)

    stop_sequences = ["\nprint", "\ndef", "\nclass", "\nif"]
    completions(
        model,
        stop_sequences,
        args.temperature,
        args.top_p,
        args.max_tokens,
        args.prompt_prefix.read_text() if args.prompt_prefix is not None else "",
        args.prompt_suffix.read_text() if args.prompt_suffix is not None else "",
        read_df(args.prompts),
        args.completions,
        args.num_completions,
    )


if __name__ == "__main__":
    main()
