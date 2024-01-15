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
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    # Add ROS2 apt repository
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    # Install ROS2 development tools
    && apt-get clean && rm -rf /var/lib/apt/lists/* && apt update \
    && apt-get upgrade -y \
    && apt-get install -y python3-colcon-mixin python3-pip python3-rosdep python3-vcstool vim tree \
        ros-humble-desktop-full ros-humble-desktop ros-humble-ros-base ros-dev-tools \
    # Add locale
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    # Install MoveIt2, see https://moveit.ros.org/install-moveit2/source/
    && apt-get install -y ros-humble-moveit

# Setup the workspace
WORKDIR /home/small-thinking/
COPY . /home/small-thinking/mnlm/

# Bootstrap rosdep
ENV ROS_DISTRO=humble \
    HOME=/home/small-thinking \
    XDG_RUNTIME_DIR=/tmp/runtime-root

WORKDIR /home/small-thinking/mnlm/

# Setup environment
RUN chmod -R 755 ./resources && ./resources/setup_env.sh \
    && sed -i 's|root:/root|root:/home/small-thinking|' /etc/passwd \
    && rosdep init && rosdep update --rosdistro $ROS_DISTRO \
    && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    # Install more packages
    && apt-get install -y ros-humble-joint-state-publisher-gui ros-humble-nav2-rviz-plugins

# # Setup colcon mixin and metadata
# RUN colcon mixin add default \
#       https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
#     colcon mixin update && \
#     colcon metadata add default \
#       https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
#     colcon metadata update