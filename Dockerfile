FROM ubuntu:jammy

WORKDIR /home/small-thinking/ros/

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    build-essential gcc locales curl git software-properties-common

RUN add-apt-repository ppa:deadsnakes/ppa

# Reset the DEBIAN_FRONTEND environment variable
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3.11 python3.11-dev python3.11-venv \
    python3.11-distutils python3.11-gdbm \
    python3.11-tk python3.11-lib2to3

# Add ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Add ROS2 apt repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt upgrade && apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools 

RUN apt-get install -y python3-colcon-mixin \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    vim

# Add locale
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8