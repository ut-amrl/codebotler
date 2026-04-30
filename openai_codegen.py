import importlib.util
import os
from pathlib import Path
from typing import List

from openai import OpenAI


TASK_PROGRAM_HEADER = "def task_program():"


def _repo_path(path: Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return Path(__file__).resolve().parent / path


def load_openai_api_key() -> str:
    key_path = Path(__file__).resolve().parent / ".openai_api_key"
    if key_path.exists():
        key = key_path.read_text().strip()
    else:
        key = os.getenv("OPENAI_API_KEY", "").strip()
    if not key:
        raise RuntimeError(
            "OpenAI API key not found. Create '.openai_api_key' in the "
            "codebotler repo or set OPENAI_API_KEY."
        )
    return key


def load_prompt_messages(prompt_path: Path) -> list[dict]:
    prompt_path = _repo_path(prompt_path)
    spec = importlib.util.spec_from_file_location("codebotler_prompt", prompt_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.messages


class OpenAICodeGenerator:
    def __init__(self, model: str, prompt_path: Path):
        print("Using OpenAI Chat model:", model)
        self.model = model
        self.client = OpenAI(api_key=load_openai_api_key())
        self.messages = load_prompt_messages(prompt_path)

    def generate_one(
        self,
        prompt: str,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ) -> str:
        if len(stop_sequences) > 4:
            raise ValueError("OpenAI API only supports up to 4 stop sequences.")

        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.messages + [{"role": "user", "content": prompt}],
            stop=stop_sequences,
            temperature=temperature,
            top_p=top_p,
            max_completion_tokens=max_tokens,
        )
        usage = response.usage
        if usage is not None:
            print(
                "Tokens used: "
                f"input={usage.prompt_tokens}, output={usage.completion_tokens}"
            )
        return ensure_task_program(response.choices[0].message.content or "")


def ensure_task_program(code: str) -> str:
    code = code.strip()
    if code.startswith(TASK_PROGRAM_HEADER):
        return code
    return f"{TASK_PROGRAM_HEADER}\n{code}".strip()
