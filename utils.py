import torch


def _stop_at_stop_token(decoded_string, stop_tokens):
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
        seq_length=1024,
        num_of_sequences=1024,
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
