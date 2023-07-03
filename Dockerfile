# Use Ubuntu 20.04 as a base with ROS Foxy installed
FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND noninteractive
ENV LANG en_US.UTF-8

# Setup locale
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install necessary tools
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    build-essential \
    git \
    software-properties-common \
    curl && \
    add-apt-repository universe && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Setup ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install --no-install-recommends -y \
    ros-foxy-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro foxy

# Create necessary directories
RUN mkdir -p /home/workspaces_art/pimouse/src

# Copy your repository into the Docker image
COPY . /home/workspaces_art/pimouse/src

# Set working directory
WORKDIR /home/workspaces_art/pimouse

# Build the workspace
RUN /bin/bash -c ". /opt/ros/foxy/setup.bash; colcon build --symlink-install"

# Update .bashrc for ROS2
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /home/workspaces_art/pimouse/install/setup.bash" >> ~/.bashrc

# Set entrypoint
CMD ["bash"]
