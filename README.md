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
All Python dependencies can be installed using:
```shell
pip install -r requirements.txt
```

**Language model options**
* To use an OpenAI model, you will need an [OpenAI key](https://platform.openai.com/account/api-keys), either saved in a file named `.openai_api_key` and placed in `robot_commands/`, or in the `OPENAI_API_KEY` environment variable.
* To use a PaLM model, you will need a [Google Generative API key](https://developers.generativeai.google/tutorials/setup), either saved in a file named `.palm_api_key` and placed in `robot_commands/`, or in the `PALM_API_KEY` environment variable.
* You can use any pretrained model compatible with the [HuggingFace AutoModel](https://huggingface.co/transformers/v3.5.1/model_doc/auto.html#automodelforcausallm) interface, including open-source models from the [HuggingFace repository](https://huggingface.co/models) such as [Starcoder](https://huggingface.co/bigcode/starcoder). Note that some models, including Starcoder, require you to agree to the HuggingFace terms of use, and you must be logged in using `huggingface-cli login`.


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
* `--robot`: Flag to indicate if the robot is available (default is `False`).

Instructions for deploying on real robots are included in [robot_interface/README.md](robot_interface/README.md).

## CodeBotler-Benchmark Quick-Start Guide

The instructions below demonstrate how to run the benchmark using the open-source [StarCoder](https://huggingface.co/bigcode/starcoder) model.

1. Run code generation for the benchmark tasks using the following command:
    ```shell
    python3 codebotler_benchmark.py --generate --model-type automodel --model-path "bigcode/starcoder" --generate-output starcoder_completions.jsonl
    ```
    This will generate the programs for the benchmark tasks and save them in
    an output file named `starcoder_completions.jsonl`. It assumes default values
    for temperature (0.2), top-p (0.9), and num-completions (20), to generate 20
    programs for each task --- this will suffice for pass@1 evaluation. 
2. Evaluate the generated programs using the following command:
    ```shell
    python3 codebotler_benchmark.py --evaluate --generate-output starcoder_completions.jsonl --evaluate-output starcoder_eval.jsonl
    ```
    This will evaluate the generated programs from the previous step, and save
    all the evaluation results in an output file named `starcoder_eval.jsonl`. It will also
    print out the pass@1 accuracy for the benchmark.

Detailed instructions for running the benchmark are included in
[benchmark/README.md](benchmark/README.md).
