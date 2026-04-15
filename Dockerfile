FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ARG ROS_DISTRO=jazzy

# Install ROS 2 Jazzy on Ubuntu 24.04.
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg2 lsb-release software-properties-common && \
    add-apt-repository universe && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop && \
    rm -rf /var/lib/apt/lists/*

  RUN apt-get update && \
    apt install git ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup -y && \
    rm -rf /var/lib/apt/lists/*

USER isaac-sim
