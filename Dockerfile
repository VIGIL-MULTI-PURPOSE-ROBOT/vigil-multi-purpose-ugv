# syntax=docker/dockerfile:1

FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-scipy \
        python3-yaml \
        novnc \
        mesa-utils \
        libqt5svg5 \
        openbox \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-interfaces \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-velocity-controllers \
        ros-jazzy-xacro \
        tini \
        websockify \
        x11vnc \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/vigil_ws
COPY ["Rock Terrain/src/", "src/"]

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
        --merge-install \
        --packages-select vigil_rough_terrain \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

COPY docker/vigil-entrypoint.sh /usr/local/bin/vigil-entrypoint
RUN chmod +x /usr/local/bin/vigil-entrypoint \
    && ln -sf vnc.html /usr/share/novnc/index.html

ENV LIBGL_ALWAYS_SOFTWARE=1 \
    QT_QPA_PLATFORM=xcb \
    ROS_DOMAIN_ID=71

EXPOSE 6080 8080

ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/vigil-entrypoint"]
CMD ["ros2", "launch", "vigil_rough_terrain", "vision_nav.launch.py", "gui:=true", "camera_width:=960", "camera_height:=720", "camera_rate:=5"]
