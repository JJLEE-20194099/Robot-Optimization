FROM --platform=linux/amd64 osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    nano python3-pip ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install scipy numpy

WORKDIR /ros2_ws
COPY ./src ./src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build

ENTRYPOINT ["/entrypoint.sh"]