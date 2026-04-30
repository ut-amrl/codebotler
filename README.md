# CodeBotler

CodeBotler is the deployment UI and OpenAI code-generation layer for turning natural-language robot tasks into Python `task_program()` functions.

## Requirements

```shell
conda create -n codebotler python=3.12.8 pip
conda activate codebotler
pip install -r requirements.txt
```

For robot deployment, source a ROS 2 workspace that provides `rclpy` and `cobot_codebotler_actions`. Store the OpenAI key in `.openai_api_key` at this repo root or set `OPENAI_API_KEY`.

## Run

Local UI without the transcript pipe:

```shell
python3 codebotler.py --disable-pipe
```

Robot deployment, matching `../../tmux/codebotler/.tmuxinator.yaml`:

```shell
python3 codebotler.py --robot --transcription-pipe audio_pipe --ip 10.1.0.13 --disable-pipe
```

The UI is served on `http://<ip>:8080/` and uses a WebSocket on port `8190`.

## Key Arguments

- `--model-name`: OpenAI chat model name, default `gpt-4`.
- `--max-tokens`: maximum generated tokens, default `512`.
- `--temperature`: OpenAI sampling temperature, default `0.2`.
- `--top-p`: OpenAI nucleus sampling value, default `0.95`.
- `--chat-prompt-prefix`: Python file containing the chat prompt `messages` list.
- `--robot`: enables ROS action execution through `robot_client.py`.
- `--disable-pipe`: disables transcript-pipe reading and uses only WebSocket/UI input.
