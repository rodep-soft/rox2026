FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ARG TARGETARCH

# 1. 重いシステム共通パッケージの事前インストール（最下層キャッシュ）
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    vim \
    curl \
    ccache \
    less \
    tree \
    tmux \
    gdb \
    htop \
    lsof \
    build-essential \
    ninja-build \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    python3-rosdep \
    iputils-ping \
    iproute2 \
    usbutils \
    can-utils \
    ros-humble-can-msgs \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-apriltag-msgs \
    ros-humble-ament-uncrustify \
    uncrustify \
    evtest \
    libboost-dev \
    libconsole-bridge-dev \
    libspdlog-dev \
    libfmt-dev \
    libyaml-cpp-dev \
    ros-humble-spdlog-vendor \
    ros-humble-console-bridge-vendor \
    ros-humble-orocos-kdl-vendor \
    ros-humble-class-loader \
    ros-humble-tf2-geometry-msgs \
    && pip3 install --no-cache-dir black cmake-format \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${TARGETARCH}" = "arm64" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
      python3-gpiozero \
      libgpiod-dev ; \
    fi && rm -rf /var/lib/apt/lists/*

RUN if [ "$TARGETARCH" = "amd64" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rqt \
      ros-humble-rqt-graph \
      ros-humble-rviz2 \
      ros-humble-foxglove-bridge; \
    fi && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

WORKDIR /root/ros2_ws

# 2. package.xml のみを先行コピーして rosdep install を完全永続キャッシュ化
# (ソースコード .cpp / .hpp が変更されても rosdep install は無駄に走らない！)
COPY ./ros2_ws/src/**/package.xml ./src_manifests/
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install \
      --from-paths src_manifests \
      --ignore-src \
      -r \
      -y && \
    rm -rf /var/lib/apt/lists/* ./src_manifests

# 3. 最後に全ソースコードをコピー
COPY ./ros2_ws/src ./src

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
