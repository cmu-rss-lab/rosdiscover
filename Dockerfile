FROM ros:indigo
RUN apt-get update \
 && apt-get install -y \
      vim python-pip
RUN pip install -U pip==18.1 \
 && pip install rosinstall-generator pyinstaller

WORKDIR /ros_ws
RUN mkdir -p /ros_ws/src \
 && rosinstall_generator turtlebot_simulator \
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


# create a portable executable
WORKDIR /opt/rosdiscover
COPY . /tmp/discover
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
      --onefile \
      --hidden-import rooibos \
      --hidden-import attrs \
 && rm -rf /tmp/* *.spec build

# # HACKS
# ENV TURTLEBOT_STAGE_MAP_FILE "foo"
# ENV TURTLEBOT_STAGE_WORLD_FILE "foo"
