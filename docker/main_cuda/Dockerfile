FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    libnvinfer8=8.6.1.6-1+cuda11.8 \
    libnvinfer-dev=8.6.1.6-1+cuda11.8 \
    libnvinfer-plugin8=8.6.1.6-1+cuda11.8 \
    libnvinfer-plugin-dev=8.6.1.6-1+cuda11.8 \
    libnvinfer-headers-dev=8.6.1.6-1+cuda11.8 \
    libnvinfer-headers-plugin-dev=8.6.1.6-1+cuda11.8 \
    libnvonnxparsers8=8.6.1.6-1+cuda11.8 \
    libnvonnxparsers-dev=8.6.1.6-1+cuda11.8 \
    libnvparsers8=8.6.1.6-1+cuda11.8 \
    libnvparsers-dev=8.6.1.6-1+cuda11.8 \
    libnvinfer-lean8=8.6.1.6-1+cuda11.8 \
    libnvinfer-dispatch8=8.6.1.6-1+cuda11.8 \
    libnvinfer-vc-plugin8=8.6.1.6-1+cuda11.8 \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-catkin-pkg \
    python3-empy \
    python3-lark \
    python3-opencv \
    python3-scipy \
    python3-matplotlib \
    git \
    zstd \
    build-essential \
    portaudio19-dev \
    ffmpeg \
    alsa-utils \
    libportaudio2 \
    libportaudiocpp0 \
    libasound2-dev \
    libglib2.0-0 \
    libgl1 \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-ros2-control \
    ros-humble-control-msgs \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-behaviortree-cpp \
    ros-humble-vision-msgs \
    ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update


COPY src /tmp/src
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install \
        --from-paths /tmp/src \
        --ignore-src \
        -r -y && \
    rm -rf /tmp/src && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "numpy<2" onnxruntime-gpu==1.16.3

COPY docker/main_cuda/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]