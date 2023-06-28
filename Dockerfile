# This is an auto-generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Clone additional repository and copy contents to /home/workspaces_art/pimouse/src/
# Create necessary directories
RUN mkdir -p /home/workspaces_art/pimouse/src

# Copy your repository into the Docker image
COPY packagesart /home/workspaces_art/pimouse/src



# Download, compile, and install the RaspberryPiMouse driver
RUN cd /home/workspaces_art/pimouse/src && \
    git clone https://github.com/rt-net/RaspberryPiMouse.git && \
    cd RaspberryPiMouse/src/driver && \
    make && \
    make install

# Set working directory
WORKDIR /home/workspaces_art/pimouse

# Continue with the rest of your Dockerfile configuration...
# Build ROS2 packages
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Source ROS2 setup.bash
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Set entrypoint
CMD ["bash"]
