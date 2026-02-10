FROM osrf/ros:humble-desktop-full

# Install Gazebo
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# Set up entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]