# %%
from transformers import (
    AutoModelForCausalLM, 
    AutoTokenizer,
    DataCollatorForLanguageModeling,
)
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
from torch.optim import Adam
from torch.utils.data import Dataset
import numpy as np
import os
import pickle
from tqdm import tqdm
import matplotlib.pyplot as plt
from pathlib import Path
import wandb
from datasets import load_dataset
import random

from generate_with_embeddings import GenerateWithEmbeddings
from utils import ConstantLengthDataset
import time
# import os
# os.environ["WANDB_MODE"] = "dryrun"

# %%
wandb.init(project="lua-training", 
           config={
                "batch_size": 1,
                "train_size_per_epoch": 100,
                "embedding_size": 64,
                "lm_prefix_size": 2048,
                "num_epochs": 400,
                "learning_rate": 3e-5,
                "checkpoint": "bigcode/santacoder",
                "revision": "dedup-alt",
                "device": "cpu",
                "time_tag": time.strftime("%Y%m%d-%H%M%S"),
                "record_step_every": 1000
           })

checkpoint = wandb.config["checkpoint"]
revision = wandb.config["revision"]
device = wandb.config["device"] 

# %%
access_token = "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"
lua_data = load_dataset("bigcode/the-stack-smol", data_dir="data/lua",split="train", use_auth_token=access_token)
lua_data.shuffle()
lua_data = lua_data.train_test_split(test_size=0.1)
train_data = lua_data["train"]
test_data = lua_data["test"]

# %%
tokenizer = AutoTokenizer.from_pretrained(checkpoint,revision=revision)
model = AutoModelForCausalLM.from_pretrained(checkpoint, revision=revision, trust_remote_code=True).to(device)
tokenizer.pad_token = tokenizer.eos_token

# %%
train_data_cl = ConstantLengthDataset(tokenizer, train_data, infinite=True, seq_length=512, num_of_sequences=512, chars_per_token=3.6)
data_collator = DataCollatorForLanguageModeling(tokenizer=tokenizer, mlm=False, return_tensors="pt")
dataloader = torch.utils.data.DataLoader(train_data_cl, 
                                         batch_size=wandb.config["batch_size"],
                                         collate_fn=data_collator)

# %%
num_epochs = wandb.config["num_epochs"]
step = 0
train_table_cols = ["epoch", "step", "loss", "label", "predictions"]
next_step_in_table = wandb.config["record_step_every"]
train_table_rows = []
train_table = wandb.Table(columns=train_table_cols)

model.train()
# %%
optimizer = torch.optim.Adam(model.parameters(), lr=wandb.config["learning_rate"])
lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=1, gamma=0.1)

time_tag = wandb.config["time_tag"]

# %%
for epoch in tqdm(range(num_epochs), desc="Epoch", position=0):
    running_loss = 0
    stats = {"correct": 0, "total": 0}
    # Select a random minibatch of examples
    indices = random.sample(range(0, len(train_data)), wandb.config["batch_size"])

    for _ in tqdm(range(wandb.config["train_size_per_epoch"]), desc="Batch", position=1):
        inputs = next(iter(dataloader))
        gwe = GenerateWithEmbeddings(model,
                                     tokenizer,
                                     None, 
                                     None,
                                     mode="train",
                                     device=device,
                                     **inputs)

        loss = gwe.generate_step()

        if step > next_step_in_table:
            next_step_in_table += wandb.config["record_step_every"]
            train_table_rows.append([epoch, step, loss.item(),"",gwe.last_predictions])
            train_table = wandb.Table(data=train_table_rows, columns=train_table_cols)
        wandb.log({"loss": loss.item(), "Training": train_table})
        step += 1

        loss.backward()
        optimizer.step()
        running_loss += loss.item()

    running_loss /= wandb.config["batch_size"]
    wandb.log({"running_loss": running_loss, "learning_rate": lr_scheduler.get_lr()[0]})
    step += 1
    lr_scheduler.step()

    if epoch % 100 == 0:
        model.save_pretrained(f"models/lua/{time_tag}_{epoch}.pt")

model.save_pretrained(f"models/lua/{time_tag}_final.pt")

# %%


