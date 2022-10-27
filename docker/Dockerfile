FROM osrf/ros:galactic-desktop-focal

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/galactic/setup.bash
RUN apt update
WORKDIR /tmp
# clone source
ENV NAO_WS /home/nao_ws
RUN mkdir -p $NAO_WS/src
WORKDIR $NAO_WS/src
RUN git clone https://github.com/AssistiveRoboticsUNH/undergraduate-ros-project.git
WORKDIR undergraduate-ros-project
RUN vcs import < external.repos.yaml

CMD ["bash"]
