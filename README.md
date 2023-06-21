# CodeBotler Overview

CodeBotler is a system that converts natural language task descriptions into
task-specific programs that can be executed by general-purpose service mobile
robots. 
It includes a benchmark for evaluating LLMs for code generation for service
mobile robot tasks. 

CodeBotler consists of two key components:
* [CodeBotler-Deploy](#codebotler-deploy-quick-start-guide): A web interface and server for deploying CodeBotler on a
  general purpose service mobile robot. You can use this to try out the code
  generation capabilities of CodeBotler either as a standalone system without a
  robot, or actually deploy it on a real robot.
* [CodeBotler-Benchmark](#codebotler-benchmark-quick-start-guide): The code generation benchmark for evaluating large language
  models (LLMs) for service mobile robot task code generation.

## Requirements
All dependencies can be installed using:
```shell
pip install -r requirements.txt
```

## CodeBotler-Deploy Quick-Start Guide

To run the web interface for CodeBotler-Deploy using the default options (using OpenAI's
`text-daVinci-003` model), run:
```shell
OPENAI_API_KEY=YOURKEY python3 codebotler_deploy.py
```
This will start the server on `localhost:8080`. You can then open the interface
by navigating to http://localhost:8080/ in your browser.  

List of arguments:
* `--ip`: The IP address to host the server on (default is `localhost`).
* `--port`: The port to host the server on (default is `8080`).
* `--ws-port`: The port to host the websocket server on (default is `8190`).
* `--model-type`: The type of model to use. It is either `openai` for [OpenAI](https://platform.openai.com) (default),
  `palm` for [PaLM](https://developers.generativeai.google/)), or `automodel`
  for
  [AutoModel](https://huggingface.co/transformers/model_doc/auto.html#automodel).
* `--model-name`: The name of the model to use. Recommended options are
  `text-daVinci-003` for OpenAI (default), `models/text-bison-001` for PaLM, and
  `bigcode/starcoder` for AutoModel.

Instructions for deploying on real robots are included in [robot_interface/README.md]

## CodeBotler-Benchmark Quick-Start Guide

The instructions below demonstrate how to run the benchmark using the open-source [StarCoder](https://huggingface.co/bigcode/starcoder) model.

1. Run code generation for the benchmark tasks using the following command:
    ```shell
    python3 codebotler_benchmark.py --generate --model-type automodel --model-path "bigcode/starcoder" --prompt-prefix benchmark/docstring_prompt_prefix.py --output starcoder_completions.json
    ```
    This will generate the programs for the benchmark tasks and save them in
    an output file named `starcoder_completions.json`. It assumes default values
    for temperature (0.2), top-p (0.9), and num-completions (20), to generate 20
    programs for each task --- this will suffice for pass@1 evaluation. 
2. Evaluate the generated programs using the following command:
    ```shell
    python3 codebotler_benchmark.py --evaluate --input starcoder_completions.json --output starcoder_eval.json
    ```
    This will evaluate the generated programs using the benchmark test, and save
    all the results in an output file named `starcoder_eval.json`. It will also
    print out the pass@1 accuracy for the benchmark.

Detailed instructions for running the benchmark are included in
[benchmark/README.md](benchmark/README.md).

## Deprecated Instructions (Merge and delete)

1. Navigate to the `code_generator` subdirectory.
2. Create a file named `.openai_api_key` with your OpenAI API key in it.
3. To start the server, run the following:
```
python interface_server.py
```
It has the following optional flags:
   - `--ip`: The IP address to host the server to (default is `localhost`).
   - `--port`: The port to host the server to (default is `8190`).
   - `--model`: It is either text-davinci-003 (default) or starcoder:
      - to specify the text-davinci-003 model, use `--model davinci`
      - to specify the starcoder model, use `--model starcoder_sip_sport` where `sip` and `sport` are the IP address and port of the starcoder server.
4. Modify the `SERVER_URL` variable in `interface.html` (if need be) to make sure it has the same `ip` and `port` as in step 3. Open the file `interface.html` in your browser.
5. Enter a task description in the text box and hit enter.
6. Once the code is generated, click on one of the buttons to indicate your
   assesment of the code's quality.
6. Repeat steps 4 and 5 for as many tasks as you want.
7. The evaluations will be saved in `eval.json`.

Example result in `eval.json`:
```json
{"type": "eval", "task": "Which room is the dog in?", "code": "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\ndog_found = False\ndog_loc = None\nfor room in list_of_rooms:\n    go_to(room)\n    if is_in_room(\"dog\"):\n        dog_found = True\n        dog_loc = room\n        break\ngo_to(start_loc)\nif dog_found:\n    say(\"The dog is in the \" + dog_loc)\nelse:\n    say(\"The dog is not in any room\")", "result": "Correct"}
{"type": "eval", "task": "Find a conference room with a projector.", "code": "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\nconf_room_found = False\nconf_room_loc = None\nfor room in list_of_rooms:\n    go_to(room)\n    if is_in_room(\"projector\"):\n        conf_room_found = True\n        conf_room_loc = room\n        break\ngo_to(start_loc)\nif conf_room_found:\n    say(\"There is a conference room with a projector in the \" + conf_room_loc)\nelse:\n    say(\"There is no conference room with a projector in the house\")", "result": "Correct"}
```
