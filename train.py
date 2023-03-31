"""
Fine-Tune SantaCoder on code/text dataset
"""

import argparse
import os

import numpy as np
import torch
from datasets import load_dataset
from torch.utils.data import IterableDataset, Dataset
from torch.utils.data.dataloader import DataLoader
from tqdm import tqdm
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    Trainer,
    TrainingArguments,
    logging,
    set_seed,
)

import fim

class CodeDataset(Dataset):
    def __init__(self, data_dir, tokenizer, prompt):
        self.data_dir = data_dir
        self.example_folders = os.listdir(self.data_dir)

        self.data = []

        # Number of prompts for in each folder
        prompt_lengths = []
        self.total_length = 0

        for d in self.example_folders:
            prompts_dir = os.path.join(self.data_dir, d, "prompts.txt")

            with open(prompts_dir, 'r') as f:
                prompts = f.read()

            prompt_length = len(prompts.split("\n"))
            prompt_lengths.append(prompt_length)

            self.total_length += prompt_length

        self.prompt_indexes = [0]
        for i in prompt_lengths[:-1]:
            self.prompt_indexes.append(self.prompt_indexes[-1] + i)

        self.DSL = prompt
        self.tokenizer = tokenizer

    def __len__(self):
        return self.total_length

    def __getitem__(self, idx):
        folder_idx = 0
        for idx, val in enumerate(self.prompt_indexes):
            if idx >= val:
                folder_idx = idx

        prompt_idx = idx - self.prompt_indexes[folder_idx]

        code_dir = os.path.join(
            self.data_dir, self.example_folders[folder_idx], "code.txt")
        prompts_dir = os.path.join(
            self.data_dir, self.example_folders[folder_idx], "prompts.txt")

        with open(code_dir, 'r') as f:
            code = f.read()

        with open(prompts_dir, 'r') as f:
            prompts = f.read().split("\n")

        code = "\n\t".join(code.split("\n"))

        final_prompt = self.DSL + prompts[prompt_idx] + "\n\t" + code
        input_ids = self.tokenizer(
            final_prompt, padding="max_length", max_length=350)["input_ids"]
        return {"input_ids": input_ids, "labels": input_ids}


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_path", type=str, default="bigcode/santacoder")
    parser.add_argument("--dataset_name", type=str,
                        default="bigcode/the-stack-dedup")
    parser.add_argument("--subset", type=str, default="data")
    parser.add_argument("--split", type=str, default="train")
    parser.add_argument("--size_valid_set", type=int, default=4000)
    parser.add_argument("--streaming", action="store_true")
    parser.add_argument("--shuffle_buffer", type=int, default=5000)
    parser.add_argument("--data_column", type=str, default="content")

    parser.add_argument("--seq_length", type=int, default=1024)
    parser.add_argument("--max_steps", type=int, default=10000)
    parser.add_argument("--batch_size", type=int, default=2)
    parser.add_argument("--gradient_accumulation_steps", type=int, default=8)
    parser.add_argument("--eos_token_id", type=int, default=49152)

    parser.add_argument("--learning_rate", type=float, default=5e-5)
    parser.add_argument("--lr_scheduler_type", type=str, default="cosine")
    parser.add_argument("--num_warmup_steps", type=int, default=100)
    parser.add_argument("--weight_decay", type=float, default=0.05)

    parser.add_argument("--local_rank", type=int, default=0)
    parser.add_argument("--no_fp16", action="store_false")
    parser.add_argument("--bf16", action="store_true")
    parser.add_argument("--no_gradient_checkpointing", action="store_false")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--num_workers", type=int, default=None)
    parser.add_argument("--output_dir", type=str, default="./checkpoints")
    parser.add_argument("--log_freq", default=1, type=int)
    parser.add_argument("--eval_freq", default=1000, type=int)
    parser.add_argument("--save_freq", default=1000, type=int)

    parser.add_argument("--fim_rate", type=float, default=0)
    parser.add_argument("--fim_spm_rate", type=float, default=0)

    # Random argument that, for some reason, is needed...
    # parser.add_argument("--local_rank", type=int, default=0)
    return parser.parse_args()


def chars_token_ratio(dataset, tokenizer, data_column, nb_examples=400):
    """
    Estimate the average number of characters per token in the dataset.
    """
    total_characters, total_tokens = 0, 0
    for _, example in tqdm(zip(range(nb_examples), iter(dataset)), total=nb_examples):
        total_characters += len(example[data_column])
        total_tokens += len(tokenizer(example[data_column]).tokens())

    return total_characters / total_tokens


class ConstantLengthDataset(torch.utils.data.IterableDataset):
    """
    Iterable dataset that returns constant length chunks of tokens from stream of text files.
        Args:
            tokenizer (Tokenizer): The processor used for proccessing the data.
            dataset (dataset.Dataset): Dataset with text files.
            infinite (bool): If True the iterator is reset after dataset reaches end else stops.
            seq_length (int): Length of token sequences to return.
            num_of_sequences (int): Number of token sequences to keep in buffer.
            chars_per_token (int): Number of characters per token used to estimate number of tokens in text buffer.
    """

    def __init__(
        self,
        tokenizer,
        dataset,
        infinite=False,
        seq_length=4,
        num_of_sequences=4,
        chars_per_token=3.6,
        content_field="content",
    ):
        self.tokenizer = tokenizer
        self.concat_token_id = (
            tokenizer.eos_token_id if tokenizer.eos_token_id else tokenizer.encode(
                "<|endoftext|>")
        )
        self.dataset = dataset
        self.seq_length = seq_length
        self.infinite = infinite
        self.current_size = 0
        self.max_buffer_size = seq_length * chars_per_token * num_of_sequences
        self.content_field = content_field

    def __iter__(self):
        iterator = iter(self.dataset)
        more_examples = True
        while more_examples:
            input_ids_buffer, labels_buffer, buffer_len = [], [], 0
            while True:
                if buffer_len >= self.max_buffer_size:
                    break
                try:
                    input_ids_buffer.append(next(iterator)["input_ids"])
                    labels_buffer.append(next(iterator)["labels"])
                    buffer_len += len(input_ids_buffer[-1])
                except StopIteration:
                    if self.infinite:
                        iterator = iter(self.dataset)
                    else:
                        more_examples = False
                        break

            all_token_ids, all_labels = [], []

            for token_ids, labels in zip(input_ids_buffer, labels_buffer):
                all_token_ids.extend(token_ids + [self.concat_token_id])
                all_labels.extend(labels + [self.concat_token_id])

            for i in range(0, len(all_token_ids), self.seq_length):
                input_ids = all_token_ids[i: i + self.seq_length]
                labels = all_labels[i: i + self.seq_length]

                if len(input_ids) == self.seq_length:
                    self.current_size += 1

                    yield {
                        "input_ids": torch.LongTensor(input_ids),
                        "labels": torch.LongTensor(labels)
                    }


def create_datasets(tokenizer, args):
    with open("DSL.txt", 'r') as f:
        prompt = f.read()

    train_dataset = CodeDataset("./train", tokenizer, prompt)
    valid_dataset = CodeDataset("./test", tokenizer, prompt)

    return train_dataset, valid_dataset


def run_training(args, train_data, val_data):
    print("Loading the model")
    # disable caching mechanism when using gradient checkpointing
    model = AutoModelForCausalLM.from_pretrained(
        args.model_path,
        trust_remote_code=True,
        use_cache=not args.no_gradient_checkpointing,
    )
    train_data.start_iteration = 0

    print(f"Starting main loop")

    training_args = TrainingArguments(
        output_dir=args.output_dir,
        dataloader_drop_last=True,
        evaluation_strategy="steps",
        max_steps=args.max_steps,
        eval_steps=args.eval_freq,
        save_steps=args.save_freq,
        logging_steps=args.log_freq,
        per_device_train_batch_size=args.batch_size,
        per_device_eval_batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        lr_scheduler_type=args.lr_scheduler_type,
        warmup_steps=args.num_warmup_steps,
        gradient_accumulation_steps=args.gradient_accumulation_steps,
        gradient_checkpointing=args.no_gradient_checkpointing,
        fp16=args.no_fp16,
        bf16=args.bf16,
        weight_decay=args.weight_decay,
        run_name=f"robot commands test 5",
        report_to="wandb",
    )

    trainer = Trainer(
        model=model, args=training_args, train_dataset=train_data, eval_dataset=val_data
    )

    print("Training...")
    trainer.train()

    print("Saving last checkpoint of the model")
    model.save_pretrained(os.path.join(args.output_dir, "final_checkpoint/"))


def main(args):
    tokenizer = AutoTokenizer.from_pretrained(
        args.model_path, use_auth_token=True)

    tokenizer.pad_token = "<pad>"
    tokenizer.padding_side = "left"

    train_dataset, eval_dataset = create_datasets(tokenizer, args)

    run_training(args, train_dataset, eval_dataset)


if __name__ == "__main__":
    args = get_args()
    set_seed(args.seed)
    os.makedirs(args.output_dir, exist_ok=True)

    logging.set_verbosity_error()

    main(args)
