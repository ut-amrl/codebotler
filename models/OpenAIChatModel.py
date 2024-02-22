from typing import List
import time
from misc.utils import load_module

class OpenAIChatModel:
    def __init__(
            self,
            model: str = None,
            api_key: str = "",
            prefix_path: str = ""):
        import openai as openai
        self.openai = openai
        print("Using OpenAI model: " + model)
        self.model = model
        self.openai.api_key = api_key
        self.structured_prefix = None
    
    def set_prefix(self, prefix_path: str):
        self.structured_prefix = load_module("prefix_msg", prefix_path).__dict__["messages"]

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        for prompt in prompts:
            while True:
                try:
                    completion = self.generate_one(prompt, stop_sequences, temperature, top_p, max_tokens)
                    break
                except self.openai.error.RateLimitError:
                    print("Rate limited...")
                    time.sleep(5)
            yield completion

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        assert self.structured_prefix is not None, "Prefix not set."
        completion = self.openai.ChatCompletion.create(
            model=self.model,
            messages=self.structured_prefix + [{"role": "user", "content": prompt}],
            temperature=temperature,
            top_p=top_p,
            stop=stop_sequences,
            max_tokens=max_tokens
        )
        code = completion.choices[0].message.content
        return code.strip()
    
    def generate_one_with_prob(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        assert self.structured_prefix is not None, "Prefix not set."
        completion = self.openai.ChatCompletion.create(
            model=self.model,
            messages=self.structured_prefix + [{"role": "user", "content": prompt}],
            temperature=temperature,
            top_p=top_p,
            stop=stop_sequences,
            max_tokens=max_tokens,
            logprobs=True,
            top_logprobs=3
        )
        code = completion.choices[0].message.content
        return code.strip(), completion.choices[0].logprobs.content