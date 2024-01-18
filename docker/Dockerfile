FROM ros:noetic
MAINTAINER gert.kanter@taltech.ee
RUN apt update && \
    apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-cv-bridge ros-noetic-xacro git
ARG CACHE_BUST
RUN cd /root && \
    git clone https://github.com/iti0201/simulation && \
    simulation/pibot_controls/install.sh
