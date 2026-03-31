FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

#install ros2 and its dep 
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    curl \
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
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros2-control \
    ros-humble-control-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

#create a tmp src to find and solve deps 
COPY src /tmp/src

#source the env and solve dep using rosdep
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install \
        --from-paths /tmp/src \
        --ignore-src \
        -r -y && \
    rm -rf /tmp/src && \
    rm -rf /var/lib/apt/lists/*

#maunallu install torch to use cache
RUN pip3 install --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cu121


#install requieremnts directlty into syustem, this container has its own requirements
COPY requirements/main.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY docker/main/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
