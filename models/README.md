# Using Different LLMs with CodeBotler+RoboEval

Instructions shown for CodeBotler --- arguments for RoboEval are the same.

## Using The HuggingFace Text Generation Inference Server

A simple wrapper script is provided to run the HuggingFace Text Generation
Inference Server at `misc/run_hf_server.sh`. You will need to set up
[podman](https://podman.io/docs/installation) (recommended) or
docker (not recommended), with [nvidia container
toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
installed.
Usage:
```bash
./misc/run-hf-tgi.sh <huggingface_hub_token> <model-name> <model-data-location> <gpus>
```

If you have errors running the server, you can inspect the logs using `podman logs <container-name>`.

### Running CodeLlama-34B Python
A script to run the Huggingface Text Generation Inference interface is provided in the `misc/` subdirectory.
Run the script with `bash codellama-tgi.sh`


In the script, set the `volume` variable to a directory. The first time the script is run, the weights will be downloaded, but on subsequent runs, the server will use the cached weights instead of re-downloading them every time. Then, once the TGI server is up, run `python codellama_completions.py`

## StarCoder
To run on a single GPU:
```bash
python3 codebotler.py --model-type automodel --model-name bigcode/starcoder
```

Using the HuggingFace Text Generation Inference server, with a dummy token
`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` (replace with an actual one!), using
`/scractch/hf_models` for model weights storage, and GPUS 0-3:
```bash
./misc/run_hf_server.sh bigcode/starcoder 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 /scratch/hf_models 0,1,2,3
python3 codebotler.py --model-type hf-textgen --model-name "http://127.0.0.1:8081/"
```

## Code Llama

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
./models/run-hf-tgi.sh x codellama/CodeLlama-34b-Python-hf /scratch/hf_tgi_models all
TODO: Update model params for codebotler.
```
