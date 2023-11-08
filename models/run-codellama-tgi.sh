#!/bin/bash

# Ensure that the user passed in the correct number of arguments
if [ "$#" -ne 5 ]; then
    echo "Usage: $0 <huggingface_hub_token> <model-name> <model-data-location> <port> <gpus>"
    echo "Example: $0 hf_00000000000000 \"codellama/CodeLlama-34b-Python-hf\" /scratch/hf_tgi_models 8081 0,1,2,3"
    exit 1
fi

HF_TOKEN=$1
MODEL_NAME=$2
MODEL_DATA_LOCATION=$3
PORT=$4
GPUS=$5

model=$MODEL_NAME
volume=$MODEL_DATA_LOCATION # share a volume with the Docker container to avoid downloading weights every run

docker run \
	-e HUGGING_FACE_HUB_TOKEN=$HF_TOKEN \
	--gpus $GPUS \
	--shm-size 1g \
	-p $PORT:80 \
	-v $volume:/data ghcr.io/huggingface/text-generation-inference:1.0.3 \
		--model-id $model \
		--max-input-length 4000 \
		--max-total-tokens 8192 \
		--max-batch-total-tokens 8192 \
		--num-shard 4 \
		--max-concurrent-requests 100 \
		--max-waiting-tokens 65536
