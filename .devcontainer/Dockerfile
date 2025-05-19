FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# ========== Basic Utilities ==========
RUN apt-get update && apt-get install -y \
    sudo build-essential \
    python3-pip \
    curl git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# ========== Create Non-root User ==========
RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ========== Back to root to install ROS CLI tools and minimal dev packages ==========
USER root

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    libusb-1.0-0-dev libv4l-dev \
    && rm -rf /var/lib/apt/lists/*

# ========== Python packages for YOLO and vision ==========
RUN pip3 install --upgrade pip && pip3 install \
    opencv-python \
    numpy \
    ultralytics

# ========== Build and Install RealSense SDK ==========
RUN apt-get update && apt-get install -y \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/IntelRealSense/librealsense.git -b v2.55.1 \
    && cd librealsense \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# ========== Finalize ==========
USER ${USERNAME}
