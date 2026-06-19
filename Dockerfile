FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

ARG TARGETARCH

# apt update & install
RUN apt-get update && apt-get install -y \
    git \
    nano \
    vim \
    curl \
    #less \
    tree \
    tmux \
    lsof \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    python3-rosdep \
    iputils-ping \
    iproute2 \
    usbutils \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    evtest \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

    #lsof is used to check which process is using the port
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
      apt-get update && apt-get install -y \
      python3-gpiozero \
      libgpiod-dev ; \
    fi && rm -rf /var/lib/apt/lists/*

RUN if [ "$TARGETARCH" = "amd64" ]; then \
      apt-get update && apt-get install -y \
      ros-jazzy-rqt \
      ros-jazzy-rqt-graph \
      ros-jazzy-rviz2 \
      ros-jazzy-foxglove-bridge \
      ccache; \
    fi && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

WORKDIR /root/ros2_ws

COPY ./ros2_ws/src ./src

# rosdep install
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install \
      --from-paths src \
      --ignore-src \
      -r \
      -y && \
      colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]