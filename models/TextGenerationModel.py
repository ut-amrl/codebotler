from text_generation import Client
from typing import List
from concurrent.futures import ThreadPoolExecutor
from models.BaseModel import BaseModel


class TextGenerationModel(BaseModel):
    def __init__(self, url, max_workers):
        self.client = Client(url, timeout=60)
        self.max_workers = max_workers

    def post_process_completion(self, completion):
        new_completion = []
        for line in completion.split("\n"):
            if "def task_program():" in line or line.startswith("    ") or line == "":
                new_completion.append(line)
            else:
                break
        return "\n".join(new_completion)

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
        return self.post_process_completion(text)

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
