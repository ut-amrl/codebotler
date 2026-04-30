# AGENTS.md

## Purpose
CodeBotler is the lean deployment layer that turns natural-language robot tasks into Python `task_program()` bodies with OpenAI chat completions, serves the browser UI/WebSocket API, and optionally executes programs through ROS 2 action clients.

## Main Flow
- `codebotler.py` is the main server. It serves `interface.html` over HTTP, accepts WebSocket messages on `--ws-port`, calls OpenAI chat completions, and optionally executes generated code.
- Generated programs must define `def task_program():` and may call only the robot helper API exposed in prompts and execution: `get_current_location`, `get_all_rooms`, `is_in_room`, `go_to`, `ask`, `say`, `pick`, and `place`.
- `codebotler.py` loads the OpenAI key from repo-root `.openai_api_key` first, then `OPENAI_API_KEY`, and loads structured few-shot messages from `openai_chat_completion_prefix.py`.
- `codebotler.py` imports and initializes `rclpy` unconditionally; if ROS 2 is unavailable, startup should fail directly. `--robot` uses real ROS action clients; without it, generated programs still execute through simulated DSL calls that each sleep for five seconds and then succeed.

## Key Files
- `codebotler.py`: HTTP/WebSocket server, OpenAI chat client, prompt loading, API-key lookup, and `task_program()` normalization.
- `interface.html`: browser UI.
- `openai_chat_completion_prefix.py`: active OpenAI chat prompt and DSL examples.
- `robot_client.py`: ROS 2 action client used by generated programs.

## Run And Verify
- Environment: `conda create -n codebotler python=3.12.8 pip`, then `pip install -r requirements.txt`.
- Local UI: `python3 codebotler.py`.
- Robot UI: source the ROS 2 workspace containing `cobot_codebotler_actions`, launch the robot action servers, then run `python3 codebotler.py --robot --ip <robot_ip>`.
- In the full workspace, run verification inside the container only: `./container cmd --name cobot_demo 'conda run --no-capture-output -n codebotler python3 -m py_compile src/codebotler/codebotler.py src/codebotler/robot_client.py src/codebotler/openai_chat_completion_prefix.py'`.
- There is no formal unit-test suite in this repo.

## Change Guidance
- Preserve the generated-code contract: output should be executable Python beginning with or containing `def task_program():`.
- Keep OpenAI stop sequences to at most four.
- Do not read, print, or commit `.openai_api_key` or ad hoc output files.
- Keep edits scoped. The codebase uses mixed indentation styles by file; follow the surrounding file instead of reformatting broadly.
- When changing ROS action usage, keep it aligned with `cobot_codebotler_actions`: `IsInRoom.Goal(object=...)`, `Pick.Goal(obj=...)`, and the action server names `/go_to_server`, `/get_current_location_server`, `/is_in_room_server`, `/say_server`, `/get_all_rooms_server`, `/ask_server`, `/pick_server`, `/place_server`.
