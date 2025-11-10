FROM osrf/ros:humble-desktop

# Install build and URDF tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    liburdfdom-tools \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy package files
COPY . .

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
