# Use Ubuntu 20.04 as a base with ROS Foxy installed
FROM ros:foxy-ros-base-focal

# Install necessary tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
# Remove the default sources list file
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
# Bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro foxy

# Setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Create necessary directories
RUN mkdir -p /home/workspaces_art/pimouse/src

# Copy your repository into the Docker image
COPY . /home/workspaces_art/pimouse/src

# Install dependencies for RaspberryPiMouse driver
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    linux-headers-$(uname -r) \
    build-essential

# Clone and install the RaspberryPiMouse driver
RUN  cd /home/workspaces_art/pimouse/src && git clone https://github.com/rt-net/RaspberryPiMouse.git && \
    cd RaspberryPiMouse/utils && \
    ./build_install.bash

# Clone and build raspimouse2 package
RUN cd /home/workspaces_art/pimouse/src && \
    git clone https://github.com/rt-net/raspimouse2.git && \
    /bin/bash -c ". /opt/ros/foxy/setup.bash; colcon build --symlink-install"

# Set working directory
WORKDIR /home/workspaces_art/pimouse

# Source ROS2 setup.bash
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source ~/workspaces_art/pimouse/install/setup.bash" >> ~/.bashrc

# Set entrypoint
CMD ["bash"]
