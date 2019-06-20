FROM ros:indigo
RUN apt-get update \
 && apt-get install -y \
      vim python-pip
RUN pip install -U pip==18.1 \
 && pip install rosinstall-generator pyinstaller

WORKDIR /ros_ws
RUN mkdir -p /ros_ws/src \
 && rosinstall_generator turtlebot_simulator rviz \
      --rosdistro indigo \
      --deps > .rosinstall \
 && wstool init -j8 /ros_ws/src /ros_ws/.rosinstall

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && apt-get update \
 && rosdep update \
 && rosdep install -i -y -r --from-paths src \
      --ignore-src \
      --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
      --rosdistro="${ROS_DISTRO}" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# add entrypoint
ENV ROS_WSPACE /ros_ws
WORKDIR "${ROS_WSPACE}"
RUN echo "#!/bin/bash \n\
set -e \n\
source \"/opt/ros/\${ROS_DISTRO}/setup.bash\" \n\
source \"${ROS_WSPACE}/devel/setup.bash\" \n\
exec \"\$@\"" > "${ROS_WSPACE}/entrypoint.sh" \
 && chmod +x "${ROS_WSPACE}/entrypoint.sh"
ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]

RUN pip install catkin_tools

# build!
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && catkin build

ENV TURTLEBOT_STAGE_MAP_FILE /ros_ws/src/turtlebot_simulator/turtlebot_stage/maps/maze.yaml
ENV TURTLEBOT_STAGE_WORLD_FILE /ros_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/maze.world

# create a portable executable
WORKDIR /opt/rosdiscover
COPY . /tmp/discover
RUN mkdir /opt/rosdiscover/lib
RUN ls /tmp/discover/
RUN cp /tmp/discover/lib/acme.standalone-ros.jar /opt/rosdiscover/lib/
RUN pip install /tmp/discover \
 && echo "\
import attr\n\
import rosdiscover.cli\n\
if __name__ == '__main__':\n\
  rosdiscover.cli.main()\
" > /tmp/rd \
 && pyinstaller --clean -y /tmp/rd \
      --name rosdiscover \
      --distpath . \
      --hidden-import rooibos \
      --hidden-import attrs \
 && rm -rf /tmp/* *.spec build
