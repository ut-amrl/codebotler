import torch


class GenerateWithEmbeddings:
    """
    A wrapper class that allows you to generate text with a transformer model, but instead of passing the input_ids to the model, you pass an embedding as prefix before the input_ids.

    Parameters:
    - model: The transformer model to use.
    - tokenizer: The tokenizer to use.
    - graph_prefix: The embedding to use as prefix.
        shape: (batch_size, 1, embedding_dim)
    - prompts: The prompt to use.
        shape: (batch_size, prompt_length)
    - device: The device to use (default: "cuda").
    - **kwargs: Any additional arguments to pass to the model.
    """
    
    def __init__(self, model, tokenizer, graph_prefix, prompts, **kwargs) -> None:
        self.model = model
        self.tokenizer = tokenizer
        self.device = kwargs.get("device", "cuda")
        self.params = kwargs
        self.last_outputs = None
        self.mode = kwargs.get("mode", "eval") 
        self.max_length = kwargs.get("max_length", 2048)
        self.graph_prefix = graph_prefix

        self.step = 0
        self.seen_stop_token = False
        self.last_predictions = []
        self.are_last_predictions_correct = None
        self.batch_size = len(prompts) if isinstance(prompts, list) else 1

        if prompts is None:
            self.input_ids = kwargs.get("input_ids", None).to(self.device)
            self.attention_mask = kwargs.get("attention_mask", None).to(self.device)
        else:
            inputs = tokenizer(prompts, return_tensors="pt", padding=True, return_token_type_ids=False).to(self.device)
            self.input_ids = inputs["input_ids"]
            self.attention_mask = inputs["attention_mask"]
        self.inputs_length = len(self.input_ids[0])

        self.inputs_embeds = self._get_embeddings(self.input_ids)

        self._validate_params()
        if graph_prefix is not None:
            self.inputs_embeds = torch.cat([graph_prefix[:, None, :], self.inputs_embeds], dim=1)
            # Pad an extra token to input_ids to compensate for addition of graph prefix.
            extra_id_padding = torch.ones(self.batch_size, 1, dtype=int, device=self.device) * self.tokenizer.pad_token_id
            self.input_ids = torch.cat([extra_id_padding, self.input_ids], dim=1)
            prefix_attention_mask = torch.ones(self.batch_size, 1,
                                               dtype=int,
                                               device=self.device)
            self.attention_mask = torch.cat([prefix_attention_mask, self.attention_mask], dim=1)
        
    def _validate_params(self):
        """
        Validate the parameters.
        """
        assert self.mode == "train" or self.mode == "eval", "mode must be either 'train' or 'eval'"
        if self.graph_prefix is not None:
            assert self.batch_size == self.graph_prefix.shape[0], f"batch_size {self.batch_size} must be equal to the batch size of the graph_prefix {self.graph_prefix.shape[0]}"

        assert not ("top_p" in self.params and "top_k" in self.params), "Cannot use both top_p and top_k"
        if "top_k" in self.params:
            assert self.params["top_k"] > 0, "top_k must be > 0"
            assert self.params["top_k"] <= self.model.config.vocab_size, "top_k must be <= vocab_size"
            assert type(self.params["top_k"]) == int, "top_k must be an integer"
        if "top_p" in self.params:
            assert 0 < self.params["top_p"] <= 1, "top_p must be > 0 and <= 1"
        
    def _get_text_tokens(self, embeds):
        return [self.tokenizer.decode(t) for t in embeds]
        
    def _next_token(self, next_tokens_logits):
        """
        Given the logits of the next token `next_token_logits`, returns a (1,) tensor containing one element representing next token.
        """

        ## TODO: Verify that this is correct with batching

        top_p = self.params.get("top_p", 0)
        top_k = self.params.get("top_k", 0)

        if top_p > 0:
            sorted_logits, sorted_indices = torch.sort(next_tokens_logits, descending=True)
            cumulative_probs = torch.cumsum(torch.nn.functional.softmax(sorted_logits, dim=-1), dim=-1)
            # Remove tokens with cumulative probability above the threshold (token with 0 are kept)
            sorted_indices_to_remove = cumulative_probs > top_p
            # Shift the indices to the right to keep also the first token above the threshold
            sorted_indices_to_remove[..., 1:] = sorted_indices_to_remove[..., :-1].clone()
            sorted_indices_to_remove[..., 0] = 0

            indices_to_remove = sorted_indices_to_remove.scatter(1, sorted_indices, sorted_indices_to_remove)
            next_tokens_logits[indices_to_remove] = float("-inf")

            return torch.multinomial(torch.nn.functional.softmax(next_tokens_logits, dim=-1), num_samples=1).reshape(-1)

        elif top_k > 0:
            indices_to_remove = next_tokens_logits < torch.topk(next_tokens_logits, top_k)[0][..., -1, None]
            next_tokens_logits[indices_to_remove] = float("-inf")
            return torch.multinomial(torch.nn.functional.softmax(next_tokens_logits, dim=-1), num_samples=1).reshape(-1)

        else: # Greedy
            return torch.argmax(next_tokens_logits, dim=-1)

    def _get_embeddings(self, input_ids):
        """
        Given a transformer, and a result produced by the tokenizer that has a sequence of "input_ids", produces
        the embedding of the tokens.
        
        The output tensor has size (BatchSize, SequenceLength, EmbeddingDim).
        
        For SantaCoder, the EmbeddingDim is 2048.
        """
        return self.model.get_input_embeddings()(input_ids)

    def generate_step(self):
        """
        Generate a step of the model.
        if prompt is not None1, it will be used as the prompt for the next step.
        otherwise, the last generated phrase (prompt + generated token) will be used as the prompt for the next step.
        """
        if not self.can_generate_more():
            raise Exception("Reached max_length tokens to generate.")

        cur_labels = self.input_ids if self.mode == "train" else None
        self.last_outputs = self.model(
            inputs_embeds=self.inputs_embeds,
            attention_mask=self.attention_mask,
            labels=cur_labels,
            output_attentions=True,
        )

        next_tokens_logits = self.last_outputs.logits[:, -1, :]
        if self.mode == "train":
            last_prediction_ids = torch.argmax(next_tokens_logits, dim=-1)
            last_predictions = self._get_text_tokens(last_prediction_ids)
            self.are_last_predictions_correct = last_prediction_ids == cur_labels[:, -1]
            # For training, we assume that there is only one call to it.
            self.seen_stop_token = True
            return self.last_outputs.loss
        else:
            next_tokens = self._next_token(next_tokens_logits)
            last_predictions = self._get_text_tokens(next_tokens)

        next_token_embeds = self._get_embeddings(next_tokens)[:, None, :]
        self.inputs_embeds = torch.cat([self.inputs_embeds, next_token_embeds], dim=1)
        self.attention_mask = torch.cat([self.attention_mask,
                                         torch.ones((self.batch_size, 1), dtype=torch.long,
                                         device=self.device)], dim=1)
        self.input_ids = torch.cat([self.input_ids, next_tokens[:, None]], dim=1)

        self.step += 1
        self.last_predictions.append(last_predictions)
    
    def generate(self):
        """
        Generate up to max_length tokens.
        """

        losses = []
        while self.can_generate_more():
            losses.append(self.generate_step())
        
        if self.mode == "train":
            return losses

    def can_generate_more(self) -> bool:
        return self.inputs_embeds.size()[1] < self.max_length and (not self.seen_stop_token)

    def get_current_texts(self) -> str | list[str]:
        """
        Get the current texts (i.e. output) of the model.
        """
        # In training mode, we don't want to include the first token, which is the padding for graph embedding.
        start_index = 1 if self.graph_prefix is not None else 0
        ret = [self.tokenizer.decode(x[start_index:]) for x in self.input_ids] 
        if len(ret) == 1:
            return ret[0]
        return ret

    def get_loss(self) -> float:
        """
        Get the loss of the model.
        """
        return self.last_outputs.loss


def predict(lm_model, tokenizer, graph_embeddings, prompts, **model_kwargs):

    generate_with_embeddings = GenerateWithEmbeddings(lm_model, tokenizer, graph_embeddings, prompts, **model_kwargs)

    generate_with_embeddings.generate()

    res = generate_with_embeddings.get_current_texts()

    return res