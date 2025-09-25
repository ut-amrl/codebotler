# =============================================================================
# Base Image and Build Arguments
#  - Docker container must match jetpack version installed on Jetson
#    - Image list: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-base/tags
#  - ROS2 Humble is used for Jetson Orin OpenCV and Python version compatibility
# =============================================================================
# JetPack container: includes CUDA, cuDNN, TensorRT, VPI, Jetson Multimedia, and so on
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
# ML container: includes TensorFlow, PyTorch, JupyterLab, and other popular ML and data science frameworks such as scikit-learn, scipy, and Pandas pre-installed in a Python environment.
#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-ml:r36.2.0-py3
FROM ${BASE_IMAGE}

ARG ros_distro=humble
ARG gid
ARG uid
ARG username

# Set non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Setup Locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Install base dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Setup Isaac ROS2 APT repository
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
  grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
  echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | tee -a /etc/apt/sources.list

# Setup Offical ROS2 APT repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install Core ROS 2 Humble packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Isaac ROS common dependencies and VPI
# Note: VPI libraries are also part of the Jetson host system and will be mounted.
# This ensures any additional dependencies are met inside the container.
RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-common \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# =============================================================================
# Realsense
# =============================================================================
#RUN apt-get update && apt-get install -y \
#  kmod \
#  && rm -rf /var/lib/apt/lists/*
#COPY scripts/install_realsense_library.sh /tmp/install_realsense_library.sh
#COPY external/librealsense /tmp/librealsense
#RUN /tmp/install_realsense_library.sh && rm /tmp/install_realsense_library.sh

# =============================================================================
# UT AUTOmata dependencies
# =============================================================================
RUN apt-get update && apt-get install -y \
  libgoogle-glog-dev \
  libgflags-dev \
  liblua5.1-0-dev \
  libqt5websockets5-dev \
  libqt5opengl5-dev \
  && rm -rf /var/lib/apt/lists/*

## =============================================================================
## GStreamer (for camera streaming)
## =============================================================================
#RUN apt-get update && apt-get install -y \
#  gstreamer1.0-plugins-base \
#  gstreamer1.0-plugins-good \
#  gstreamer1.0-plugins-bad \
#  gstreamer1.0-plugins-ugly \
#  gstreamer1.0-libav \
#  gstreamer1.0-tools \
#  gstreamer1.0-x \
#  && rm -rf /var/lib/apt/lists/*

# =============================================================================
# EGL, OpenGL libraries (for GUI applications)
# =============================================================================
RUN apt-get update && apt-get install -y \
  libegl1 \
  libegl1-mesa \
  libgles2-mesa \
  libgles2 \
  libapr1-dev \
  libglvnd-dev \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  mesa-utils-extra \
  && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Install Kinect SDK
# =============================================================================
RUN apt-get update && apt-get install -y \
  libgtk-3-dev \
  build-essential \
  git \
  cmake \
  libssl-dev \
  libsoundio-dev \
  libjpeg-dev \
  libturbojpeg-dev \
  ninja-build \
  && rm -rf /var/lib/apt/lists/*

# Install Deps
# Define variables
ENV SDK_VERSION="1.4.2"
ENV ARCH="arm64"
ENV DEB_PACKAGE="libk4a1.4_${SDK_VERSION}_${ARCH}.deb"
ENV DEB_URL="https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4/${DEB_PACKAGE}"

# Install proprietary depth engine
RUN mkdir -p /tmp/k4a_deps && cd /tmp/k4a_deps && \
    wget ${DEB_URL} && \
    ar x "${DEB_PACKAGE}" && \
    ls -lah . && \
    tar -zxf data.tar.gz && \
    # Copy the depthengine to the Azure Kinect build directory
    cp /tmp/k4a_deps/usr/lib/aarch64-linux-gnu/libk4a1.4/libdepthengine.so.2.0 /usr/local/lib/ && \
    ldconfig && \
    # Clean up
    cd /tmp && \
    rm -rf /tmp/k4a_deps

# Build SDK from source
RUN cd /tmp && \
    git clone --branch v1.4.2 https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git --recurse-submodules
COPY patches/kinect-hmac-fix.patch /tmp/Azure-Kinect-Sensor-SDK/kinect-hmac-fix.patch
COPY patches/kinect-ssl.patch /tmp/Azure-Kinect-Sensor-SDK/kinect-ssl.patch
COPY patches/ebml.patch /tmp/Azure-Kinect-Sensor-SDK/patches/ebml.patch
COPY patches/k4a.patch /tmp/Azure-Kinect-Sensor-SDK/patches/k4a.patch
COPY patches/examples.patch /tmp/Azure-Kinect-Sensor-SDK/patches/examples.patch
RUN openssl version
RUN cd /tmp/Azure-Kinect-Sensor-SDK && \
    cp scripts/99-k4a.rules /etc/udev/rules.d/ && \
    patch -p1 -i kinect-hmac-fix.patch && \
    patch -p1 -i kinect-ssl.patch && \
    patch -p1 -i patches/ebml.patch && \
    patch -p1 -i patches/k4a.patch && \
    patch -p1 -i patches/examples.patch && \
    head -n20 tools/k4aviewer/k4adepthpixelcolorizer.h && \
    mkdir build && cd build && \
    cmake -GNinja -DCMAKE_C_FLAGS="-Wno-deprecated-declarations -Wno-error=discarded-qualifiers" .. && \
    ninja && \
    sudo ninja install && \
    rm -rf /tmp/Azure-Kinect-Sensor-SDK

COPY scripts/ros2_nav_deps.sh /tmp/ros2_nav_deps.sh
RUN /tmp/ros2_nav_deps.sh && rm /tmp/ros2_nav_deps.sh

COPY scripts/ros2_deps.sh /tmp/ros2_deps.sh
RUN /tmp/ros2_deps.sh && rm /tmp/ros2_deps.sh

COPY scripts/install_livox_lidar.sh /tmp/install_livox_lidar.sh
RUN /tmp/install_livox_lidar.sh && rm /tmp/install_livox_lidar.sh

# =============================================================================
# TODO: move up deep learning deps
# =============================================================================
RUN pip3 install torch==2.6.0 --index-url https://download.pytorch.org/whl/cu126

# So we can detect if we are in a container
ENV CONTAINER_NAME=codebotler

# =============================================================================
# Install Kinova python API
# =============================================================================
RUN python3 -m pip install --no-cache-dir \
    protobuf==3.20.0 \
    deprecated==1.2.7 \
    paho-mqtt==1.6.1 \
    mypy \
    mypy-protobuf \
    types-protobuf 
RUN wget https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/kortex_api-2.5.0.post6-py3-none-any.whl \
    && python3 -m pip install --no-deps kortex_api-2.5.0.post6-py3-none-any.whl \
    && rm kortex_api-2.5.0.post6-py3-none-any.whl
RUN apt-get update && apt-get install -y --no-install-recommends \
      libhidapi-hidraw0 \
      libhidapi-libusb0 \
      libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir hidapi

# =============================================================================
# Spacemouse/Joystick Depdencies
# =============================================================================
RUN apt-get update && apt-get install -y \ 
    libhidapi-hidraw0 \
    joystick \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Kinova Vision dependencies
# ============================================================================
RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-base \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Kinova ROS dependencies
# ============================================================================
RUN apt-get update && apt-get install -y \
    libcap-dev \
    libdw-dev \
    && rm -rf /var/lib/apt/lists/*

# remove, if installed, the pointcloud_to_laserscan package
RUN apt-get remove --purge -y ros-$ROS_DISTRO-pointcloud-to-laserscan || true

# =============================================================================
# User Creation and Configuration
# =============================================================================
ENV UID=$uid
ENV GID=$gid
ENV USERNAME=$username
ENV ROS_DISTRO=$ros_distro

# Create user and group, then configure sudo
RUN set -e && \
    # Create group if it doesn't exist
    getent group ${GID} || groupadd -g ${GID} ${USERNAME} && \
    # Handle user creation/modification
    if id ${UID} >/dev/null 2>&1; then \
        # User with this UID exists, modify it
        existing_user=$(id -nu ${UID}) && \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m $existing_user 2>/dev/null || true && \
        usermod -g ${GID} ${USERNAME} 2>/dev/null || true; \
    else \
        # Create new user
        useradd -u ${UID} -g ${GID} -m -s /bin/bash ${USERNAME}; \
    fi && \
    # Ensure home directory exists and has correct ownership
    mkdir -p /home/${USERNAME} && \
    chown -R ${UID}:${GID} /home/${USERNAME} && \
    # Add to sudo group and configure passwordless sudo
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    visudo -c && \
    # Add the user to the video group for access to video devices
    usermod -aG video ${USERNAME}

RUN echo "export PATH=/home/${USERNAME}/.local/bin/:${PATH}" >> /home/${USERNAME}/.bash_profile && \
    echo "export LC_ALL=C.UTF-8" >> /home/${USERNAME}/.bash_profile && \
    echo "export LANG=C.UTF-8" >> /home/${USERNAME}/.bash_profile && \
    echo "alias killros2='killall -9 ros2 _ros2_daemon rviz2 gzserver robot_state_publisher gzclient controller_server lifecycle_manager'" >> /home/${USERNAME}/.bash_profile && \
    echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\" --" >> /home/${USERNAME}/.bash_profile && \
    echo "source \"/home/${USERNAME}/cobot_ws/install/setup.bash\" --" >> /home/${USERNAME}/.bash_profile && \
    echo "export ROS_VERSION=2" >> /home/${USERNAME}/.bash_profile && \
    echo "export PS1='\\[\\e[0;32m\\][docker]:\\u@\\h:\\w\\$ \\[\\e[0m\\]'" >> /home/${USERNAME}/.bash_profile && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/torch/lib:\$LD_LIBRARY_PATH" >> /home/${USERNAME}/.bash_profile && \
    echo "cd /home/${USERNAME}/cobot_ws" >> /home/${USERNAME}/.bash_profile


# =============================================================================
# Switch to User Context
# =============================================================================

USER $username
ENV HOME=/home/$username
WORKDIR /home/$username

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
