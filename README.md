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

Local UI:

```shell
python3 codebotler.py
```

Without `--robot`, generated programs still execute through simulated DSL calls. Each DSL call blocks for five seconds, emits the same UI status updates, then succeeds.

Robot deployment, matching `../../tmux/codebotler/.tmuxinator.yaml`:

```shell
python3 codebotler.py --robot --ip 10.1.0.13
```

The UI is served on `http://<ip>:8080/` and uses a WebSocket on port `8190`.

## Key Arguments

- `--model-name`: OpenAI chat model name, default `gpt-5.4-mini`.
- `--max-tokens`: maximum generated tokens, default `512`.
- `--temperature`: OpenAI sampling temperature, default `0.2`.
- `--top-p`: OpenAI nucleus sampling value, default `0.95`.
- `--robot`: uses real ROS action clients instead of simulated DSL calls.
