FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/home/ardupilot

# -----------------------------
# System dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    git \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    build-essential \
    ccache \
    gdb \
    cmake \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Install MAVProxy
# -----------------------------
RUN pip3 install mavproxy

# -----------------------------
# Create a non-root user
# -----------------------------
RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ardupilot
WORKDIR /home/ardupilot

# -----------------------------
# ArduPilot setup
# -----------------------------
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/ardupilot/ardupilot
RUN git submodule update --init --recursive

# Waf alias
RUN echo "alias waf=\"$PWD/modules/waf/waf-light\"" >> ~/.bashrc

# Run prerequisites
RUN ./Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build SITL
RUN ./waf configure --board=sitl
RUN ./waf all

# -----------------------------
# ArduPilot_Gazebo setup
# -----------------------------
RUN git clone https://github.com/SwiftGust/ardupilot_gazebo.git
WORKDIR /home/ardupilot/ardupilot_gazebo
RUN mkdir build
WORKDIR /home/ardupilot/ardupilot_gazebo/build
RUN cmake ..
RUN make -j4
RUN sudo make install

# -----------------------------
# Environment setup for VS Code terminals
# -----------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/ardupilot/ardupilot_gazebo/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/ardupilot/ardupilot_gazebo/models_gazebo:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=/home/ardupilot/ardupilot_gazebo/worlds:\${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=/home/ardupilot/ardupilot_gazebo/build:\${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc

WORKDIR /home/ardupilot
ENTRYPOINT ["/bin/bash"]