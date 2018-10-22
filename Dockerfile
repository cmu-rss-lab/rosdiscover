FROM ros:indigo
RUN apt-get update \
 && apt-get install -y \
      vim python-pip
RUN pip install -U pip==18.1 \
 && pip install rosinstall-generator
COPY . /tmp/discover
RUN cd /tmp/discover \
 && pip install . \
 && rm -rf /tmp/*

WORKDIR /ros_ws
RUN mkdir -p /ros_ws/src \
 && rosinstall_generator turtlebot_simulator \
      --rosdistro indigo \
      --deps > .rosinstall \
 && wstool init -j8 /ros_ws/src /ros_ws/.rosinstall
