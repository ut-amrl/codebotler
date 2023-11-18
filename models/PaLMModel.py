from typing import List
from models.BaseModel import BaseModel
import time


class PaLMModel(BaseModel):
    def __init__(self, model: str, api_key: str):
        import google.generativeai as palm
        from google.generativeai.types.safety_types import (
            HarmCategory,
            HarmBlockThreshold,
        )

        self.safety_setting = [
            {
                "category": HarmCategory.HARM_CATEGORY_TOXICITY,
                "threshold": HarmBlockThreshold.BLOCK_NONE,
            }
        ]
        self.palm = palm
        palm.configure(api_key=api_key)
        self.model = model

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        for prompt in prompts:
            completion = self.palm.generate_text(
                model=self.model,
                prompt=prompt,
                temperature=temperature,
                stop_sequences=stop_sequences,
                top_p=top_p,
                max_output_tokens=max_tokens,
                safety_settings=self.safety_setting,
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
        max_tokens: int,
    ):
        return self.palm.generate_text(
            model=self.model,
            prompt=prompt,
            temperature=temperature,
            stop_sequences=stop_sequences,
            top_p=top_p,
            max_output_tokens=max_tokens,
        ).result
