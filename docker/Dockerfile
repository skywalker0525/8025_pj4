FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-msgs \
    tmux \
    ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir fastapi==0.110.0 "uvicorn[standard]==0.27.1"

RUN rosdep init || true

COPY docker/ros_env.sh /etc/profile.d/ros_env.sh
COPY docker/entrypoint.sh /ros_entrypoint_overlay.sh
COPY docker/tmux-dev.sh /usr/local/bin/tmux-dev
RUN chmod +x /ros_entrypoint_overlay.sh /etc/profile.d/ros_env.sh /usr/local/bin/tmux-dev \
 && printf '\nsource /etc/profile.d/ros_env.sh\n' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint_overlay.sh"]
CMD ["bash"]
