FROM ubuntu:jammy

# Set shell for running commands
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    build-essential gcc locales curl git software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa

# Reset the DEBIAN_FRONTEND environment variable
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3.11 python3.11-dev python3.11-venv \
    python3.11-distutils python3.11-gdbm \
    python3.11-tk python3.11-lib2to3

# Add ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Add ROS2 apt repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 development tools
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt update \
    && apt upgrade \
    && apt install -y \
    ros-humble-desktop ros-humble-ros-base ros-dev-tools \
    python3-colcon-mixin python3-pip python3-rosdep python3-vcstool vim tree

# Add locale
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Install Gazebo, see https://gazebosim.org/docs/harmonic/install_ubuntu.
RUN apt-get update && apt-get install -y lsb-release wget gnupg qtcreator qtbase5-dev qt5-qmake cmake \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y gz-harmonic


# Install MoveIt2, see https://moveit.ros.org/install-moveit2/source/.
RUN apt-get install -y ros-humble-moveit

# Setup the workspace
WORKDIR /home/small-thinking/
COPY . /home/small-thinking/mnlm/

# Setup environment
RUN chmod -R 755 ./mnlm/resources && ./mnlm/resources/setup_env.sh \
    && sed -i 's|root:/root|root:/home/small-thinking|' /etc/passwd

# Bootstrap rosdep
ENV ROS_DISTRO=humble \
    HOME=/home/small-thinking

WORKDIR /home/small-thinking/mnlm/
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update