from typing import List
from models.BaseModel import BaseModel
import time


class OpenAIChatModel(BaseModel):
    def __init__(self, model: str = None, api_key: str = ""):
        import openai as openai

        self.openai = openai
        print("Using OpenAI model: " + model)
        self.model = model
        self.openai.api_key = api_key
        from code_generation.openai_chat_completion_prefix import messages

        self.structured_prefix = messages

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        assert (
            len(stop_sequences) <= 4
        ), "OpenAI API only supports up to 4 stop sequences."
        for prompt in prompts:
            while True:
                try:
                    completion = self.generate_one(
                        prompt, stop_sequences, temperature, top_p, max_tokens
                    )
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
        max_tokens: int,
    ):
        assert (
            len(stop_sequences) <= 4
        ), "OpenAI API only supports up to 4 stop sequences."
        completion = self.openai.ChatCompletion.create(
            model=self.model,
            messages=self.structured_prefix + [{"role": "user", "content": prompt}],
            temperature=temperature,
            top_p=top_p,
            stop=stop_sequences,
            max_tokens=max_tokens,
        )
        return completion.choices[0].message.content
