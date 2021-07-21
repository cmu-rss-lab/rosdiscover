FROM autoware/autoware:1.6.0-indigo

RUN cd ~/Autoware && git checkout fc8f691c70bf62a3de0f7448d9490b2e9385e6c4


RUN sudo apt-get update 
RUN sudo apt-get install -y build-essential checkinstall
RUN sudo apt-get install -y libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev
RUN cd /usr/src && sudo wget https://www.python.org/ftp/python/2.7.18/Python-2.7.18.tgz && sudo tar xzf Python-2.7.18.tgz && cd Python-2.7.18 && sudo ./configure --enable-optimizations && sudo make altinstall

RUN sudo apt update 
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py | python2.7
RUN python2.7 -m pip install -U wstools

RUN sudo apt-get update \
 && sudo apt-get install -y --no-install-recommends \
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
      "ros-indigo-cmake-modules" \
      software-properties-common \
      tmux \
      vim \
      wget 

 RUN sudo apt-get clean \
 && sudo apt-get update && sudo apt-get upgrade -y \
 && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo wget http://packages.ros.org/ros.key -O - | sudo apt-key add - \
 && sudo apt-get update \ 
 && sudo apt-get install python-rosinstall -y \
 && python2.7 -m pip install \
      catkin-tools==0.4.5 \
      coverage==5.1 \
      rosinstall-generator==0.1.18 \
 && sudo apt-get clean \
 && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update && sudo apt-get update \
&& sudo apt-get install -y python-coverage \
 python-catkin-pkg \
 python-rosdep \
 python-rosdistro \
 python-rosinstall \
 python-rosinstall-generator \
 python-rospkg \
 python-vcstools \
 python-wstool 

COPY ./* /install/

RUN sudo wstool init -j8 src_pre_bug /install/pre_bug.rosinstall \
&& rosdep update \
&& rosdep install -i -y -r --from-paths src_pre_bug \
      --ignore-src \
      --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
      --rosdistro="${ROS_DISTRO}" 

RUN sudo wstool init -j8 src_bug /install/bug.rosinstall \
&& rosdep install -i -y -r --from-paths src_bug \
      --ignore-src \
      --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
      --rosdistro="${ROS_DISTRO}" 

RUN sudo wstool init -j8 src_bug_fix /install/bug_fix.rosinstall \
&& rosdep install -i -y -r --from-paths src_bug_fix \
      --ignore-src \
      --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
      --rosdistro="${ROS_DISTRO}" 

# install vncserver
RUN sudo apt-get update \
 && export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get install -y \
      supervisor \
      vnc4server \
      xfce4 \
      xfce4-goodies \
      xfce4-terminal \
      xserver-xorg-core \
      xterm \
      xvfb \
      x11vnc \
 && sudo apt-get clean \
 && sudo rm -rf /var/lib/apt/lists/* \
 && sudo mkdir ~/.vnc \
 && sudo /bin/bash -c "echo -e 'r0sD!sc0ver\nr0sD!sc0ver\nn' | vncpasswd"
ENV TINI_VERSION v0.9.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /bin/tini
RUN sudo chmod +x /bin/tini

COPY rootfs /
