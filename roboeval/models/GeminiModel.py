from typing import List
import time
from google import genai
from google.genai import types

from tqdm import tqdm

class GeminiModel:
    def __init__(
            self,
            model: str = None,
            api_key: str = ""):
        self.client = genai.Client(api_key=api_key)

        print("Using Gemini model: " + model)
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
            while True:
                try:
                    completion = self.generate_one(prompt, stop, temperature, top_p, max_tokens)
                    outputs.append(completion)
                    break
                except Exception as e:
                    print(f"Error generating completion: {e}. Sleeping for 10s...")
                    time.sleep(10)
        return outputs

    def generate_one(
        self,
        prompt: str,
        stop: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int):
        completion = self.client.models.generate_content(
            model=self.model,
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction='generate program that completes the instruction after `def task_program():`',
                max_output_tokens= max_tokens,
                top_p=top_p,
                temperature=temperature,
            )
        )
        code = completion.text
        
        return code.strip()

