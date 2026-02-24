FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# ROS + Gazebo packages
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    gazebo11 \
    libgazebo11-dev \
    git \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    build-essential \
    ccache \
    gdb \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# MAVProxy
# -----------------------------
RUN pip3 install mavproxy

# -----------------------------
# ArduPilot
# -----------------------------
WORKDIR /opt
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR /opt/ardupilot

# Submodules
RUN git submodule update --init --recursive

# Install ArduPilot prerequisites
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build SITL
RUN ./waf configure --board=sitl
RUN ./waf all

# -----------------------------
# ardupilot_gazebo (SwiftGust)
# -----------------------------
WORKDIR /opt
RUN git clone https://github.com/SwiftGust/ardupilot_gazebo.git
WORKDIR /opt/ardupilot_gazebo
RUN mkdir build
WORKDIR /opt/ardupilot_gazebo/build
RUN cmake ..
RUN make -j4
RUN make install

# -----------------------------
# Source ROS automatically
# -----------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# Gazebo environment setup
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ardupilot_gazebo/models:\${GAZEBO_MODEL_PATH}" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ardupilot_gazebo/models_gazebo:\${GAZEBO_MODEL_PATH}" >> /root/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=/opt/ardupilot_gazebo/worlds:\${GAZEBO_RESOURCE_PATH}" >> /root/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=/opt/ardupilot_gazebo/build:\${GAZEBO_PLUGIN_PATH}" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]