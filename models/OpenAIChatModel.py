from typing import List
import time
from misc.utils import load_module
from openai import OpenAI  # new client-based API

class OpenAIChatModel:
    def __init__(self, model: str = None, api_key: str = "", prefix_path: str = ""):
        print("Using OpenAI Chat model:", model)
        self.model = model
        self.client = OpenAI(api_key=api_key)
        self.structured_prefix = None

    def set_prefix(self, prefix_path: str):
        self.structured_prefix = load_module("prefix_msg", prefix_path).__dict__["messages"]

    def generate(self, prompts: list, stop_sequences: List[str],
                 temperature: float, top_p: float, max_tokens: int):
        if len(stop_sequences) > 4:
            raise ValueError("OpenAI API only supports up to 4 stop sequences.")
        for prompt in prompts:
            # You might implement retry/backoff here if desired.
            while True:
                try:
                    completion = self.generate_one(prompt, stop_sequences,
                                                   temperature, top_p, max_tokens)
                    break
                except Exception as e:
                    print("Error during generation:", e)
                    raise
            yield completion

    def generate_one(self, prompt: str, stop_sequences: List[str],
                     temperature: float, top_p: float, max_tokens: int):
        if len(stop_sequences) > 4:
            raise ValueError("OpenAI API only supports up to 4 stop sequences.")
        if self.structured_prefix is None:
            raise ValueError("Prefix not set.")
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.structured_prefix + [{"role": "user", "content": prompt}],
            stop=stop_sequences,
            max_completion_tokens=max_tokens
        )
        print(f"Tokens used: input={response.usage.prompt_tokens}, output={response.usage.completion_tokens}, completion_details={response.usage.completion_tokens_details}, prompt_details={response.usage.prompt_tokens_details}")
        return response.choices[0].message.content.strip()

    def generate_one_with_prob(self, prompt: str, stop_sequences: List[str],
                               temperature: float, top_p: float, max_tokens: int):
        if len(stop_sequences) > 4:
            raise ValueError("OpenAI API only supports up to 4 stop sequences.")
        if self.structured_prefix is None:
            raise ValueError("Prefix not set.")
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.structured_prefix + [{"role": "user", "content": prompt}],
            stop=stop_sequences,
            max_completion_tokens=max_tokens,
            logprobs=True,       # request log probabilities (if supported)
            top_logprobs=3
        )
        return response.choices[0].message.content.strip(), response.choices[0].logprobs.content
