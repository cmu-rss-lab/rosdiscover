# This is a generic Dockerfile that is used to build all of the images within
# the dataset. To do so, the Dockerfile relies on the use of build arguments
# and the copying of additional scripts from the directory of the robot that
# is being built.
#
# Build arguments
# ---------------
# BUILD_COMMAND: the command that should be used to build the Catkin workspace.
#   By default, catkin_make is used, but one may also use "catkin build"
#   instead as a means of avoiding certain build issues.
# DISTRO: specifies the ROS distribution that should be used by the base image
# DIRECTORY: specifies the directory that provides files for the robot
# GZWEB: use to specify whether GzWeb support should be added to the image
#   values should be "yes" or "no".
#
# References
# ----------
# * http://blog.fx.lv/2017/08/running-gui-apps-in-docker-containers-using-vnc
# * https://qxf2.com/blog/view-docker-container-display-using-vnc-viewer
# * https://github.com/ConSol/docker-headless-vnc-container/blob/master/src/ubuntu/install/libnss_wrapper.sh
# * https://www.digitalocean.com/community/tutorials/how-to-install-and-configure-vnc-on-ubuntu-18-04
# * https://hackernoon.com/installation-of-vnc-server-on-ubuntu-1cf035370bd3
# * https://www.tecmint.com/install-and-configure-vnc-server-on-ubuntu
# * https://stackoverflow.com/questions/48601146/docker-how-to-set-tightvncserver-password
ARG DISTRO

FROM ubuntu:18.04 AS gzweb
ENV GZWEB_RELEASE 5781220babcd95b217b28e69f6f8b6542be014b9
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ca-certificates \
      git \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && git clone https://github.com/osrf/gzweb /opt/gzweb -u "${GZWEB_RELEASE}"

# NOTE: We need to install the cmake_modules to avoid some build failures
# that are due to unspecified dependencies.
# (https://github.com/ros-industrial/industrial_calibration/issues/50)
# supervisor
FROM ros:${DISTRO} AS main
ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
COPY --from=gzweb /opt/gzweb /opt/gzweb
COPY rootfs /
WORKDIR /ros_ws
ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      apt-utils \
      bzip2 \
      cmake \
      clang \
      build-essential \
      ca-certificates \
      curl \
      gcc \
      g++ \
      mercurial \
      python-pip \
      "ros-${ROS_DISTRO}-cmake-modules" \
      software-properties-common \
      tmux \
      vim \
      wget \
 && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list \
 && wget http://packages.ros.org/ros.key -O - | apt-key add - \
 && apt-get update \
 && pip install \
      catkin-tools==0.4.5 \
      coverage==5.1 \
      rosinstall-generator==0.1.18 \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# install gzweb deps
ENV NODE_PATH /opt/nodejs
ENV NODE_VERSION v6.17.1
ENV NODE_DISTRO linux-x64
ENV NODE_RELEASE "node-${NODE_VERSION}-${NODE_DISTRO}"
ENV PATH "${NODE_PATH}/${NODE_RELEASE}/bin:${PATH}"
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      imagemagick \
      libjansson-dev \
      libboost-dev \
      libtinyxml-dev \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && cd /tmp \
 && wget -nv "https://nodejs.org/dist/${NODE_VERSION}/${NODE_RELEASE}.tar.xz" \
 && mkdir -p "${NODE_PATH}" \
 && tar -xJvf "${NODE_RELEASE}.tar.xz" -C "${NODE_PATH}" \
 && rm -f "${NODE_RELEASE}.tar.xz"

# install vncserver
RUN apt-get update \
 && export DEBIAN_FRONTEND=noninteractive \
 && apt-get install -y \
      supervisor \
      vnc4server \
      xfce4 \
      xfce4-goodies \
      xfce4-terminal \
      xserver-xorg-core \
      xterm \
      xvfb \
      x11vnc \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir ~/.vnc \
 && /bin/bash -c "echo -e 'password\npassword\nn' | vncpasswd"
ENV TINI_VERSION v0.9.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /bin/tini
RUN chmod +x /bin/tini

# build package
ARG DIRECTORY
COPY "${DIRECTORY}"/pkgs.rosinstall "${DIRECTORY}"/apt.list* /.dockerinstall/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && apt-get update \
 && (test -f /.dockerinstall/apt.list \
     && xargs -a /.dockerinstall/apt.list apt-get install -y --no-install-recommends \
     || exit 0) \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN mv /.dockerinstall/pkgs.rosinstall /ros_ws/pkgs.rosinstall \
 && apt-get update \
 && wstool init -j8 src pkgs.rosinstall \
 && rosdep update \
 && rosdep install -i -y -r --from-paths src \
      --ignore-src \
      --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
      --rosdistro="${ROS_DISTRO}" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

COPY "${DIRECTORY}" /.dockerinstall/
RUN ls -al /.dockerinstall
RUN (test -f /.dockerinstall/prebuild.sh \
     && ((echo "running prebuild step..." && apt-get update && /bin/bash /.dockerinstall/prebuild.sh && apt-get clean && rm -rf /var/lib/apt/lists/* )|| exit 1) \
     || (echo "skipping prebuild step [no prebuild.sh]" && exit 0))

ARG BUILD_COMMAND="catkin_make"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && eval "${BUILD_COMMAND}"

# optionally add gzweb support
ARG GZWEB="no"
RUN (test "${GZWEB}" = "yes" \
     && (echo "running gzweb installation scripts..." && /.dockerinstall/install-gzweb.sh || exit 1) \
     || (echo "skipping gzweb installation step" && exit 0))
