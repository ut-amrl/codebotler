from text_generation import Client


def print_by_line(previous_text: str, new_text: str):
    """
    A little hack to print line-by-line in a Notebook. We receive results
    a few tokens at a time. This buffers output until a newline, so that
    we do not print partial lines.
    """
    if "\n" not in new_text:
        return
    last_newline = previous_text.rfind("\n")
    if last_newline != -1:
        print(previous_text[last_newline + 1:] + new_text, end="")
    else:
        print(previous_text + new_text, end="")


def generate(client: Client, prompt: str, max_new_tokens, repetition_penalty, return_full_text, stop_sequences, temperature, top_p, echo):
    text = ""
    for response in client.generate_stream(
        prompt,
        max_new_tokens=max_new_tokens,
        repetition_penalty=repetition_penalty,
        return_full_text=return_full_text,
        stop_sequences=stop_sequences,
        temperature=temperature,
        top_p=top_p
    ):
        if not response.token.special:
            if echo:
                print_by_line(text, response.token.text)
            text += response.token.text
    if echo:
        print_by_line(text, "\n")  # flush any remaining text
    return text


def convert_to_list(input_data):
    if isinstance(input_data, list):
        return input_data
    else:
        return [input_data]


def get_code_string(ip, port, prompt, temperature, max_tokens, top_p, repetition_penalty, stop, echo=False, return_full_text=False):
    URL = f"http://{ip}:{port}"
    client = Client(URL)
    return generate(
        client=client,
        prompt=prompt,
        max_new_tokens=max_tokens,
        repetition_penalty=repetition_penalty,
        return_full_text=return_full_text,
        stop_sequences=convert_to_list(stop),
        temperature=temperature,
        top_p=top_p,
        echo=echo
    )
