#!/bin/bash
docker run -it --rm \
    --gpus all \
    -v .:/home/codebotler \
    -w /home/codebotler \
    --network host \
    zichaoatut/codebotler-ros2-humble:v1.0 bash