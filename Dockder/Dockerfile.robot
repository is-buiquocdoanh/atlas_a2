FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── Hệ thống & ROS2 packages ──────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    can-utils \
    usbutils \
    udev \
    # ROS2 nav stack
    ros-humble-nav2-bringup \
    ros-humble-nav2-collision-monitor \
    ros-humble-nav2-map-server \
    ros-humble-nav2-velocity-smoother \
    ros-humble-slam-toolbox \
    # Robot drivers & sensors
    ros-humble-rplidar-ros \
    ros-humble-v4l2-camera \
    ros-humble-robot-state-publisher \
    ros-humble-twist-mux \
    ros-humble-twist-mux-msgs \
    ros-humble-topic-tools \
    # Image pipeline
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-web-video-server \
    # TF & msgs
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-msgs \
  && rm -rf /var/lib/apt/lists/*

# ── Python deps ───────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    ultralytics \
    flask \
    flask-cors \
    websockets \
    numpy \
    pyserial \
    python-can

# ── Copy & build workspace ────────────────────────────────────────────────────
WORKDIR /workspace
COPY src/ src/

RUN source /opt/ros/humble/setup.bash \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths src --ignore-src -r -y \
 && colcon build --symlink-install \
      --packages-ignore atlas_app \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build/

# ── Maps volume ───────────────────────────────────────────────────────────────
RUN mkdir -p /workspace/src/atlas_base/atlas_maps
VOLUME ["/workspace/src/atlas_base/atlas_maps"]

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY Dockder/entrypoint_robot.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
