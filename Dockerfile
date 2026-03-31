FROM --platform=linux/amd64 osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    nano python3-pip ros-humble-foxglove-bridge \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 uninstall -y numpy
RUN pip3 install "numpy<2.0" scipy opencv-python cv-bridge

WORKDIR /ros2_ws
COPY ./src ./src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build

ENTRYPOINT ["/entrypoint.sh"]