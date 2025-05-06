FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# 安装基本依赖
RUN apt-get update && apt-get install -y \
    sudo build-essential \
    python3-pip \
    curl git lsb-release gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# 创建用户
RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

# 自动 source ROS 环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 切回 root 权限安装
USER root

# 安装 librealsense 源码构建依赖
RUN apt-get update && apt-get install -y \
    git cmake build-essential libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libudev-dev libopencv-dev libv4l-dev


RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl gnupg2 lsb-release

# 切换用户（必须单独一行）
USER ${USERNAME}



