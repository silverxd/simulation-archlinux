FROM ros:noetic
MAINTAINER gert.kanter@taltech.ee
RUN apt update && \
    apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-cv-bridge git && \
    cd /root && \
    git clone https://github.com/iti0201/simulation && \
    simulation/pibot_controls/install.sh
