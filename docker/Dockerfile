FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies...
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    git \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /katana_ws

# Because context is '..', '.' copies ALL modules inside katana_driver_ros2/
COPY . /katana_ws/src/katana_driver_ros2/

# Install rosdep dependencies
SHELL ["/bin/bash", "-c"]
RUN apt-get update && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Copy entrypoint script and fix line endings
# (Assuming the entrypoint script is inside the docker folder)
RUN cp /katana_ws/src/katana_driver_ros2/docker/docker-entrypoint.sh /docker-entrypoint.sh && \
    sed -i 's/\r$//' /docker-entrypoint.sh && \
    chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "katana400_moveit_config", "full_system.launch.py", "connection_type:=tcp", "ip_address:=192.168.1.1"]
