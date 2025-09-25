# =============================================================================
# Base Image and Build Arguments
#  - Docker container must match jetpack version installed on Jetson
#    - Image list: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-base/tags
#  - ROS2 Humble is used for Jetson Orin OpenCV and Python version compatibility
#  - This image builds off of the cobot_ws image
# =============================================================================
# JetPack container: includes CUDA, cuDNN, TensorRT, VPI, Jetson Multimedia, and so on
FROM cobot_ws AS base

ARG uid
ARG gid
ARG username
USER $username

# Set environment variable for Miniconda installation directory
ENV CONDA_DIR /home/$username/.miniconda3

# Download and install Miniconda
RUN wget --quiet wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p ${CONDA_DIR} && \
    rm /tmp/miniconda.sh

ENV PATH=$CONDA_DIR/bin:$PATH

WORKDIR /home/$username
COPY src/codebotler /home/$username/codebotler
WORKDIR /home/$username/codebotler
RUN conda env create -f conda_environment.yaml
RUN conda clean -afy
ENTRYPOINT ["conda", "run", "python", "codebotler.py"]
