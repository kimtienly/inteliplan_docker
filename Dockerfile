# FROM ros:noetic-perception
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive


ENV ROS_WS_DIR="/inteliplan_ws"

WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]


# Update package lists and install dependencies
RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS package source and keys
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list && \
    curl -sSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" | apt-key add -

# Update and install ROS Noetic base
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up the ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO=noetic
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV PATH="/opt/ros/noetic/bin:$PATH"

# Install additional dependencies (Optional)
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Default command
CMD ["/bin/bash"]


# Useful non-ROS tools
RUN apt-get update \
 && apt-get install -y \
    git \
    net-tools \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Useful ROS tools
RUN apt-get update \
 && apt-get install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-vcstool \
    locales \
 && rm -rf /var/lib/apt/lists/*

# Set up locale to avoid the keyboard layout prompt
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# Taken from HSR wiki page
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list' \
 && sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list' \
 && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add - \
 && wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add - \
 && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && sh -c 'mkdir -p /etc/apt/auth.conf.d' \
 && sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf' \
 && apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-tmc-desktop-full \
 && rm -rf /var/lib/apt/lists/*
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-hsrb-description \
    ros-${ROS_DISTRO}-tmc-control-msgs \
 && rm -rf /var/lib/apt/lists/*

# install nano for editing within docker if needed
RUN apt-get update 
RUN apt-get install nano -y

# install orion packages: tmux and openni2-camera package
RUN apt-get update \
 && apt-get install -y \
    python3-pip \
    tmux \
    openni2-utils \
    portaudio19-dev \
    python3-pyaudio \
    python3-sphinx \
 && rm -rf /var/lib/apt/lists/*

RUN pip install numpy
RUN pip install numba --upgrade
RUN pip install requests --upgrade

# Semantic mapping dependencies
RUN apt-get update
RUN apt-get install mongodb-server-core -y
RUN pip install pymongo

# Manipulation dependencies
RUN pip install imutils
RUN apt install ros-noetic-octomap 
# RUN apt install ros-noetic-octomap-server # cannot build from docker
RUN apt install ros-noetic-octomap-ros
RUN apt install ros-noetic-rviz-visual-tools -y

# Navigation dependencies
RUN pip install transform3d

# Perception dependencies
RUN pip install einops
RUN pip install seaborn

RUN pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116

# install caffe (needed for gpd)
RUN sudo cp /etc/apt/sources.list /etc/apt/sources.list~
RUN sudo sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list
RUN sudo apt-get update
RUN apt build-dep caffe-cpu -y
RUN apt install libatlas-base-dev -y

ENV DEBIAN_FRONTEND=dialog


# ---------------------------------------------------------------
# Use login shell to read variables from `~/.profile` (to pass dynamic created variables between RUN commands)
SHELL ["sh", "-lc"]

# The following `ARG` are mainly used to specify the versions explicitly & directly in this docker file, and not meant
# to be used as arguments for docker build (so far).

ARG PYTORCH='2.1.0'
# (not always a valid torch version)
ARG INTEL_TORCH_EXT='2.1.0'
# Example: `cu102`, `cu113`, etc.
ARG CUDA='cu118'

RUN apt update && apt install -y git libsndfile1-dev tesseract-ocr espeak-ng python3 python3-pip ffmpeg git-lfs
RUN git lfs install
RUN python3 -m pip install --no-cache-dir --upgrade pip

# ARG REF=main
# RUN git clone https://github.com/huggingface/transformers && cd transformers && git checkout $REF

# TODO: Handle these in a python utility script
RUN [ ${#PYTORCH} -gt 0 -a "$PYTORCH" != "pre" ] && VERSION='torch=='$PYTORCH'.*' ||  VERSION='torch'; echo "export VERSION='$VERSION'" >> ~/.profile
RUN echo torch=$VERSION

RUN [ "$PYTORCH" != "pre" ] && python3 -m pip install --no-cache-dir -U $VERSION torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/$CUDA || python3 -m pip install --no-cache-dir -U --pre torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/nightly/$CUDA

RUN python3 -m pip install --no-cache-dir -U tensorflow==2.13 protobuf==3.20.3 tensorflow_text tensorflow_probability

# RUN python3 -m pip install --no-cache-dir -e ./transformers[dev,onnxruntime]
RUN python3 -m pip install transformers==4.44.2

RUN python3 -m pip uninstall -y flax jax

RUN python3 -m pip install --no-cache-dir intel_extension_for_pytorch==$INTEL_TORCH_EXT -f https://developer.intel.com/ipex-whl-stable-cpu

RUN python3 -m pip install --no-cache-dir git+https://github.com/facebookresearch/detectron2.git pytesseract
RUN python3 -m pip install -U "itsdangerous<2.1.0"

# RUN python3 -m pip install --no-cache-dir git+https://github.com/huggingface/accelerate@main#egg=accelerate
RUN python3 -m pip install accelerate

# RUN python3 -m pip install --no-cache-dir git+https://github.com/huggingface/peft@main#egg=peft
RUN python3 -m pip install peft

# Add bitsandbytes for mixed int8 testing
RUN python3 -m pip install --no-cache-dir bitsandbytes

# Add auto-gptq for gtpq quantization testing
RUN python3 -m pip install --no-cache-dir auto-gptq --extra-index-url https://huggingface.github.io/autogptq-index/whl/cu118/

# Add einops for additional model testing
RUN python3 -m pip install --no-cache-dir einops

# Add autoawq for quantization testing
# RUN python3 -m pip install --no-cache-dir https://github.com/casper-hansen/AutoAWQ/releases/download/v0.1.6/autoawq-0.1.6+cu118-cp38-cp38-linux_x86_64.whl

# For bettertransformer + gptq 
# RUN python3 -m pip install --no-cache-dir git+https://github.com/huggingface/optimum@main#egg=optimum
RUN python3 -m pip install optimum
# For video model testing
RUN python3 -m pip install --no-cache-dir decord av==9.2.0

RUN python3 -m pip install markupsafe==2.0.1
RUN python3 -m pip install trl==0.10.1
RUN python3 -m pip install jinja2==3.1.2
RUN python3 -m pip install wand
RUN python3 -m pip install numpy==1.22