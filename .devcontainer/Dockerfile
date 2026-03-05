FROM osrf/ros:humble-desktop-full

# ---------------------------------------------------------------------------
# 1. Gazebo Classic 11 + ROS2 packages
#    osrf/ros:humble-desktop-full ships Gazebo Classic 11 via the OSRF repo.
#    libgazebo-dev is the correct package name on Ubuntu Jammy (no version suffix).
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. General build dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git \
    gitk \
    git-gui \
    curl \
    cmake \
    build-essential \
    sudo \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-yaml \
    libxml2-dev \
    libxslt1-dev \
    libbz2-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libgps-dev \
    libcap-dev \
    zip \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. MAVProxy + pymavlink
# ---------------------------------------------------------------------------
RUN pip3 install MAVProxy pymavlink future

# ---------------------------------------------------------------------------
# 4. Non-root user required by ArduPilot's prereq script
# ---------------------------------------------------------------------------
RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ---------------------------------------------------------------------------
# 5. ArduPilot SITL
# ---------------------------------------------------------------------------
# Clone ArduPilot with a larger HTTP buffer and retry logic to handle
# flaky connections during the large submodule fetch.
RUN git config --global http.postBuffer 524288000 && \
    git config --global http.lowSpeedLimit 0 && \
    git config --global http.lowSpeedTime 999999 && \
    until git clone --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /ardupilot; do \
        echo "Clone failed, retrying in 10s..."; \
        rm -rf /ardupilot; \
        sleep 10; \
    done && \
    chown -R ardupilot:ardupilot /ardupilot

USER ardupilot
WORKDIR /ardupilot

RUN USER=ardupilot Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build SITL for copter (multi-drone) and plane
RUN ./waf configure --board sitl && \
    ./waf copter && \
    ./waf plane

USER root

# Fix git ownership warning — /ardupilot was cloned as ardupilot user but
# will be used as root, so we need to mark it as a safe directory.
RUN git config --global --add safe.directory /ardupilot

# ---------------------------------------------------------------------------
# 6. ardupilot_gazebo for Gazebo Classic — khancyr fork
#
#    ArduPilot/ardupilot_gazebo (main branch) targets Gazebo Garden/Harmonic
#    and requires gz-cmake3, which is NOT present in ROS Humble's Gazebo Classic.
#
#    khancyr/ardupilot_gazebo is the canonical legacy plugin for Gazebo Classic
#    (Gazebo 9–11). It uses a plain cmake build with sudo make install.
# ---------------------------------------------------------------------------
RUN git clone https://github.com/khancyr/ardupilot_gazebo.git /ardupilot_gazebo

RUN cd /ardupilot_gazebo && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j$(nproc) && \
    make install

# ---------------------------------------------------------------------------
# 7. Environment variables
# ---------------------------------------------------------------------------
ENV ARDUPILOT_HOME=/ardupilot
ENV PATH="${PATH}:/ardupilot/Tools/autotest"
ENV GAZEBO_MODEL_PATH=/ardupilot_gazebo/models
ENV GAZEBO_RESOURCE_PATH=/ardupilot_gazebo/worlds
# Plugin is installed system-wide via make install — no GAZEBO_PLUGIN_PATH needed

# ---------------------------------------------------------------------------
# 8. Shell setup for both users
# ---------------------------------------------------------------------------
RUN for RCFILE in /root/.bashrc /home/ardupilot/.bashrc; do \
    echo "source /opt/ros/humble/setup.bash" >> $RCFILE; \
    echo "source /usr/share/gazebo/setup.bash" >> $RCFILE; \
    echo "export PATH=\$PATH:/ardupilot/Tools/autotest" >> $RCFILE; \
    echo "export GAZEBO_MODEL_PATH=/ardupilot_gazebo/models" >> $RCFILE; \
    echo "export GAZEBO_RESOURCE_PATH=/ardupilot_gazebo/worlds:\${GAZEBO_RESOURCE_PATH}" >> $RCFILE; \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> $RCFILE; \
    echo "mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root" >> $RCFILE; \
    done

COPY .devcontainer/setup-display.sh /usr/local/bin/setup-display.sh
RUN chmod +x /usr/local/bin/setup-display.sh

WORKDIR /workspace
ENTRYPOINT ["/bin/bash"]