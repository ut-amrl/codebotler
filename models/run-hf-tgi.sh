#!/bin/bash

# Ensure that the user passed in the correct number of arguments
if [ "$#" -ne 5 ]; then
    echo "Usage: $0 <huggingface_hub_token> <model-name> <model-data-location> <port> <gpus>"
    echo "Example starcoder: $0 hf_00000000000000 \"bigcode/starcoder\" /scratch/hf_tgi_models 8081 0,1,2,3"
    echo "Example codellama 34b: $0 hf_00000000000000 \"codellama/CodeLlama-34b-Python-hf\" /scratch/hf_tgi_models 8082 4,5,6,7"
    exit 1
fi

HF_TOKEN=$1
MODEL_NAME=$2
MODEL_DATA_LOCATION=$3
PORT=$4
GPUS=$5

podman run --gpus $GPUS \
        --shm-size 1g \
        -p $PORT:80 \
        --restart=no \
        --log-opt path=/tmp/hf-tgi-podman.log \
        -v $MODEL_DATA_LOCATION:/data \
        -e HUGGING_FACE_HUB_TOKEN=$HF_TOKEN \
        -d \
        ghcr.io/huggingface/text-generation-inference:1.0.3 \
                --model-id $MODEL_NAME \
                --max-input-length 4000 \
                --max-total-tokens 8192 \
                --max-batch-total-tokens 8192 \
                --num-shard 4 \
                --max-concurrent-requests 100 \
                --max-waiting-tokens 65536
