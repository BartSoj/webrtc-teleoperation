FROM ros:humble AS base

SHELL ["/bin/bash", "-c"]

WORKDIR /root/ws

RUN apt-get update && apt-get install -y \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev \
    libavformat-dev \
    git \
    cmake \
    gdb \
    build-essential \
    pkg-config \
    nlohmann-json3-dev \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS
RUN source /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /root/ws/install/setup.bash" >> ~/.bashrc

# Setup entrypoint
COPY docker/entrypoint.sh /
RUN sudo chmod 0755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN mkdir -p external/libdatachannel \
    && git clone https://github.com/paullouisageneau/libdatachannel.git /root/ws/external/libdatachannel \
    && cd /root/ws/external/libdatachannel \
    && git submodule update --init --recursive --depth 1 \
    && cmake -B build -DUSE_GNUTLS=0 -DUSE_NICE=0 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cd build \
    && make -j2 \
    && make install

COPY . /root/ws/src/webrtc_teleop

# Build teleop_client_lib
RUN mkdir -p /root/ws/build/teleop_client_lib && \
    cd /root/ws/build/teleop_client_lib && \
    cmake /root/ws/src/webrtc_teleop/teleop_client_lib && \
    make -j2 && \
    make install && \
    ldconfig