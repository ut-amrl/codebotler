#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

pip install -r requirements.txt

# Initialize Conda in this script
eval "$(/opt/miniconda3/bin/conda shell.bash hook)"  # This is the recommended way to initialize Conda in scripts
# deactivate conda env for building
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    echo "No Conda environment is currently active."
    CURRENT_ENV=""
else
    CURRENT_ENV="$CONDA_DEFAULT_ENV"
    echo "Deactivating Conda environment: $CURRENT_ENV"
    conda deactivate
fi
rm -rf build/ devel/ || true
catkin_make
catkin_make  # sanity check
# Reactivate the previous Conda environment if it was active
if [ -z "$CURRENT_ENV" ]; then
    echo "No Conda environment was previously active. Skipping activation."
else
    echo "Reactivating Conda environment: $CURRENT_ENV"
    conda activate "$CURRENT_ENV"
fi