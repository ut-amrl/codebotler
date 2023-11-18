# Using Different LLMs with CodeBotler+RoboEval

Instructions shown for CodeBotler --- arguments for RoboEval are the same.

## Using The HuggingFace Text Generation Inference Server

A simple wrapper script is provided to run the HuggingFace Text Generation
Inference Server at `models/run_hf_server.sh`. You will need to set up
[podman](https://podman.io/docs/installation) (recommended) or
docker (not recommended), with [nvidia container
toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
installed.
Usage:
```bash
bash model/run-hf-tgi.sh <huggingface_hub_token> <model-name> <model-data-location> <port> <gpus>
```

If you have errors running the server, you can inspect the logs using `podman logs <container-name>`.

## StarCoder
To run on a single GPU:
```bash
python3 codebotler.py --model-type automodel --model-name bigcode/starcoder
```

Using the HuggingFace Text Generation Inference server, with a dummy token
`hf_00000000000000` (replace with an actual one!), using
`/scractch/hf_models` for model weights storage, and GPUS 0-3:
```bash
./misc/run_hf_server.sh hf_00000000000000 "bigcode/starcoder" /scratch/hf_models 8081 0,1,2,3
python3 codebotler.py --model-type hf-textgen --model-name "http://127.0.0.1:8081/"
```

## CodeLlama

Code Llama requires the bleeding edge version of `accelerate` and `transformers`:
```bash
pip install git+https://github.com/huggingface/transformers.git@main  git+https://github.com/huggingface/accelerate.git@main
```

To run on a single GPU:
```bash
python3 codebotler.py --model-type automodel --model-name codellama/CodeLlama-7b-Python-hf
```

Using the HuggingFace Text Generation Inference server:
```
bash models/run-hf-tgi.sh hf_00000000000000 "codellama/CodeLlama-34b-Python-hf" /scratch/hf_tgi_models 8081 0,1,2,3
```
