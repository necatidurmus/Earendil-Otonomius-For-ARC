FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    terminator \
    vim \
    wget \
    x11-apps \
    # Gazebo + ROS-GZ bridge
    ros-humble-ros-gz \
    mesa-utils \
    locales \
    # Navigation & SLAM
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    # Perception
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-opencv \
    # Teleop
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    # Rosbag
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN mkdir -p /home/$USERNAME/ws && chown $USERNAME:$USERNAME /home/$USERNAME/ws

USER $USERNAME
WORKDIR /home/$USERNAME/ws

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "test -f /home/$USERNAME/ws/install/setup.bash && source /home/$USERNAME/ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
