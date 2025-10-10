# CodeBotler

CodeBotler is a system that converts natural language task descriptions into robot-agnostic programs that can be executed by general-purpose service mobile robots.

![CodeBotler Web Interface](https://amrl.cs.utexas.edu/codebotler/assets/images/et_gif.gif)

## Requirements

We provide a conda environment to run our code. To create and activate the environment:
```shell
conda create -n codebotler python=3.12.8 pip
conda activate codebotler
pip install -r requirements.txt
```
After installing the conda environment, please go to [pytorch's official website](https://pytorch.org/get-started/locally/) to install the pytorch corresponding to your cuda version (**Note: do not install the cpu version**).

**ROS2 Requirements**
* For robot deployment, you will need ROS2 installed on your system. CodeBotler uses ROS2 actions for robot communication.
* Install ROS2 following the [official ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html).
* The robot interface components will automatically install the required ROS2 Python packages (`rclpy`).

**Language Model Options**
* To use an OpenAI model, you will need an [OpenAI key](https://platform.openai.com/account/api-keys), either saved in a file named `.openai_api_key`, or in the `OPENAI_API_KEY` environment variable.
* To use a PaLM model, you will need a [Google Generative API key](https://developers.generativeai.google/tutorials/setup), either saved in a file named `.palm_api_key`, or in the `PALM_API_KEY` environment variable.
* You can use any pretrained model compatible with the [HuggingFace AutoModel](https://huggingface.co/transformers/v3.5.1/model_doc/auto.html#automodelforcausallm) interface, including open-source models from the [HuggingFace repository](https://huggingface.co/models) such as [Starcoder](https://huggingface.co/bigcode/starcoder). Note that some models, including Starcoder, require you to agree to the HuggingFace terms of use, and you must be logged in using `huggingface-cli login`.
* You can also use a [HuggingFace Inference Endpoint](https://huggingface.co/docs/inference-endpoints/index).

## Quick Start Guide

To run the web interface for CodeBotler using the default options (using OpenAI's `gpt-4` model), run:
```shell
python3 codebotler.py
```
This will start the server on `localhost:8080`. You can then open the interface by navigating to http://localhost:8080/ in your browser.

### Arguments
* `--ip`: The IP address to host the server on (default is `localhost`).
* `--port`: The port to host the server on (default is `8080`).
* `--ws-port`: The port to host the websocket server on (default is `8190`).
* `--model-type`: The type of model to use. It is either `openai-chat` (default) and `openai` for [OpenAI](https://platform.openai.com), `palm` for [PaLM](https://developers.generativeai.google/), or `automodel` for [AutoModel](https://huggingface.co/transformers/model_doc/auto.html#automodel).
* `--model-name`: The name of the model to use. Recommended options are `gpt-4` for GPT-4 (default), `text-daVinci-003` for GPT-3.5, `models/text-bison-001` for PaLM, and `bigcode/starcoder` for AutoModel.
* `--robot`: Flag to indicate if the robot is available (default is `False`).
* `--transcription-pipe`: Path to the named pipe for recieving transcription information

Instructions for deploying on real robots are included in [robot_interface/README.md](robot_interface/README.md).
