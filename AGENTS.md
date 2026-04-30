# AGENTS.md

## Purpose
CodeBotler is the lean deployment layer that turns natural-language robot tasks into Python `task_program()` bodies with OpenAI chat completions, serves the browser UI/WebSocket API, and optionally executes programs through ROS 2 action clients.

## Main Flow
- `codebotler.py` is the main server. It serves `code_generation/interface.html` over HTTP, accepts WebSocket messages on `--ws-port`, calls `openai_codegen.py`, and optionally executes generated code.
- Generated programs must define `def task_program():` and may call only the robot helper API exposed in prompts and execution: `get_current_location`, `get_all_rooms`, `is_in_room`, `go_to`, `ask`, `say`, `pick`, and `place`.
- `openai_codegen.py` loads the OpenAI key from repo-root `.openai_api_key` first, then `OPENAI_API_KEY`, and loads structured few-shot messages from `code_generation/openai_chat_completion_prefix.py`.
- Robot execution is enabled only with `--robot` and a working `rclpy` import. `robot_client.py` creates action clients and runs generated code in a temporary subprocess with the helper functions bound as locals.
- The audio pipe path defaults to `/tmp/audio_pipe`. Final transcript lines beginning with `<FINAL>:` are accumulated from wake word `cobot` through end word `done`; saying `yes execute please` executes the most recently generated program.

## Key Files
- `openai_codegen.py`: OpenAI chat client, prompt loading, API-key lookup, and `task_program()` normalization.
- `code_generation/`: the active OpenAI chat prompt and browser UI.
- `robot_client.py`: ROS 2 action client used by generated programs.
- `cli.py`: WebSocket client for scripted prompt generation and optional execution.

## Run And Verify
- Environment: `conda create -n codebotler python=3.12.8 pip`, then `pip install -r requirements.txt`.
- Local UI without the audio pipe: `python3 codebotler.py --disable-pipe`.
- Robot UI: source the ROS 2 workspace containing `cobot_codebotler_actions`, launch the robot action servers, then run `python3 codebotler.py --robot --ip <robot_ip>`.
- CLI smoke: `python3 cli.py --host localhost --port 8190 "Go to the kitchen."`.
- In the full workspace, run verification inside the container only: `./container cmd --name cobot_demo 'conda run --no-capture-output -n codebotler python3 -m py_compile src/codebotler/codebotler.py src/codebotler/cli.py src/codebotler/openai_codegen.py src/codebotler/robot_client.py'`.
- There is no formal unit-test suite in this repo.

## Change Guidance
- Preserve the generated-code contract: output should be executable Python beginning with or containing `def task_program():`.
- Keep OpenAI stop sequences to at most four.
- Do not read, print, or commit `.openai_api_key`, generated audio, or ad hoc output files.
- Keep edits scoped. The codebase uses mixed indentation styles by file; follow the surrounding file instead of reformatting broadly.
- When changing ROS action usage, keep it aligned with `cobot_codebotler_actions`: `IsInRoom.Goal(object=...)`, `Pick.Goal(obj=...)`, and the action server names `/go_to_server`, `/get_current_location_server`, `/is_in_room_server`, `/say_server`, `/get_all_rooms_server`, `/ask_server`, `/pick_server`, `/place_server`.
