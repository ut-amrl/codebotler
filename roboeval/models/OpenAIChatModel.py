from typing import List
import time
from openai import OpenAI
from tqdm import tqdm

class OpenAIChatModel:
    def __init__(
            self,
            model: str = None,
            api_key: str = "",
            prefix_path: str = ""):
        self.client = OpenAI(api_key=api_key)
        print("Using OpenAI model: " + model)
        self.model = model

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        outputs = []
        for prompt in tqdm(prompts):
            completion = self.generate_one(prompt, stop_sequences, temperature, top_p, max_tokens)
            outputs.append(completion)
        return outputs

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        assert len(stop_sequences) <= 4, "OpenAI API only supports up to 4 stop sequences."
        
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=prompt,
            temperature=temperature,
            top_p=top_p,
            max_tokens=max_tokens
        )
        code = completion.choices[0].message.content
        return code.strip()