from typing import List
import time
from roboeval.misc.utils import load_module
from openai import OpenAI
from tqdm import tqdm

class OpenAIChatModel:
    def __init__(
            self,
            model: str = None,
            api_key: str = ""):
        self.client = OpenAI(api_key=api_key)
        print("Using OpenAI model: " + model)
        self.model = model

    def generate(
        self,
        prompts: list,
        stop: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop) <= 4, "OpenAI API only supports up to 4 stop sequences."
        outputs = []
        for prompt in tqdm(prompts):
            completion = self.generate_one(prompt, stop, temperature, top_p, max_tokens)
            outputs.append(completion)
        return outputs

    def generate_one(
        self,
        prompt: str,
        stop: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop) <= 4, "OpenAI API only supports up to 4 stop sequences."
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=prompt,
            temperature=temperature,
            top_p=top_p,
            max_tokens=max_tokens
        )
        code = completion.choices[0].message.content
        return code.strip()