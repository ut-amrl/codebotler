from typing import List, Union

class BaseModel:
    def __init__(self):
        """ Initializes the model. """
        assert("This should be implemented in child classes!")

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        """ Yields completions given a sequence of prompts. """
        assert("This should be implemented in child classes!")

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        """ Returns a single completion given a single prompt. """
        assert("This should be implemented in child classes!")
