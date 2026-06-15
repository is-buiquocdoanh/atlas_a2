FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── System & Qt deps ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # PyQt5 & display
    python3-pyqt5 \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    # ROS2 msgs needed by app
    ros-humble-nav2-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy requests

# ── Copy & build workspace (chỉ build atlas_app) ─────────────────────────────
WORKDIR /workspace
COPY src/ src/

RUN source /opt/ros/humble/setup.bash \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths src/atlas_base/atlas_app \
                               src/atlas_base/atlas_slam \
                   --ignore-src -r -y \
 && colcon build --symlink-install \
      --packages-select atlas_app atlas_slam \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build/

COPY Dockder/entrypoint_app.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
