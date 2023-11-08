from typing import List, Union
from models.BaseModel import BaseModel
import time


class OpenAIModel(BaseModel):
    def __init__(
        self,
        use_azure: bool = False,
        engine: Union[str, None] = None,
        model: Union[str, None] = None,
        api_base: Union[str, None] = None,
        api_version: Union[str, None] = None,
        api_key: str = "",
    ):
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
        max_tokens: int,
    ):
        assert (
            len(stop_sequences) <= 4
        ), "OpenAI API only supports up to 4 stop sequences."
        for prompt in prompts:
            kwargs = {
                "prompt": prompt,
                "temperature": temperature,
                "max_tokens": max_tokens,
                "top_p": top_p,
                "stop": stop_sequences,
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
        max_tokens: int,
    ):
        assert (
            len(stop_sequences) <= 4
        ), "OpenAI API only supports up to 4 stop sequences."
        kwargs = {
            "prompt": prompt,
            "temperature": temperature,
            "max_tokens": max_tokens,
            "top_p": top_p,
            "stop": stop_sequences,
        }
        if self.engine is not None:
            kwargs["engine"] = self.engine
        elif self.model is not None:
            kwargs["model"] = self.model
        return self.openai.Completion.create(**kwargs)["choices"][0]["text"]
