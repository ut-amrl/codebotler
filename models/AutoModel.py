from transformers import AutoTokenizer, AutoModelForCausalLM
from models.BaseModel import BaseModel
from typing import List


def stop_at_stop_token(decoded_string, stop_tokens):
    """
    Produces the prefix of decoded_string that ends at the first occurrence of
    a stop_token.

    WARNING: the decoded_string *must not* include the prompt, which may have stop tokens
    itself.
    """
    min_stop_index = len(decoded_string)
    for stop_token in stop_tokens:
        stop_index = decoded_string.find(stop_token)
        if stop_index != -1 and stop_index < min_stop_index:
            min_stop_index = stop_index
    return decoded_string[:min_stop_index]


class AutoModel(BaseModel):
    def __init__(self, batch_size, path):
        import torch

        self.batch_size = batch_size
        self.model = AutoModelForCausalLM.from_pretrained(
            path, trust_remote_code=True, torch_dtype=torch.bfloat16
        ).cuda()
        self.tokenizer = AutoTokenizer.from_pretrained(
            path, trust_remote_code=True, padding_side="left"
        )
        if (
            ("starchat" in path)
            or ("starcoder" in path)
            or ("santacoder" in path)
            or ("xgen" in path)
        ):
            self.tokenizer.pad_token = self.tokenizer.eos_token
        elif "llama" in path:
            self.tokenizer.pad_token = "<|endoftext|>"

    def generate_batch(
        self,
        prompts,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        encoded_prompts = self.tokenizer(
            [p.rstrip() for p in prompts],
            padding=True,
            return_attention_mask=True,
            return_tensors="pt",
            return_token_type_ids=False,
        ).to(0)
        max_input_tokens = encoded_prompts["input_ids"].shape[1]
        outputs = self.model.generate(
            **encoded_prompts,
            do_sample=True,
            top_p=top_p,
            temperature=temperature,
            max_length=max_tokens + max_input_tokens,
            pad_token_id=self.tokenizer.pad_token_id
        )
        decoded_outputs = self.tokenizer.batch_decode(
            outputs[:, max_input_tokens:],
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )
        return [stop_at_stop_token(s, stop_sequences) for s in decoded_outputs]

    def generate(
        self,
        prompts: list,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        for i in range(0, len(prompts), self.batch_size):
            batch = prompts[i : i + self.batch_size]
            for completion in self.generate_batch(
                batch, stop_sequences, temperature, top_p, max_tokens
            ):
                yield completion

    def generate_one(
        self,
        prompt,
        stop_sequences: List[str],
        temperature: float,
        top_p: float,
        max_tokens: int,
    ):
        encoded_prompt = self.tokenizer.encode(
            prompt.rstrip(),
            padding=True,
            return_attention_mask=True,
            return_tensors="pt",
        ).to(0)
        num_input_tokens = encoded_prompt.shape[1]
        output = self.model.generate(
            encoded_prompt,
            do_sample=True,
            top_p=top_p,
            temperature=temperature,
            max_length=max_tokens + num_input_tokens,
        )
        decoded_output = self.tokenizer.decode(
            output[0, num_input_tokens:],
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )
        return stop_at_stop_token(decoded_output, stop_sequences)
