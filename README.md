# rosdiscover


## Installation

See below for two different methods of installing rosdiscover.
In general, the native installation should be preferred, but for some cases
(e.g., machines that run Mac OS or Windows), the Docker-based method is
ideal.

Both methods require that Docker is installed on your machine and that your
user belongs to the `docker` group (i.e., `sudo` isn't required to run `docker`
commands).
For instructions on installing Docker, refer to: https://docs.docker.com/install/


### Native Installation

The ideal way to install `rosdiscover` is to install to a virtual environment
running on your host machine. This method requires that your host machine is
running Python 3.6 or greater.

We strongly recommend that you install `rosdiscover` inside a Python virtual
environment (via virtualenv or pipenv) to avoid interfering with the rest of
your system (i.e., to avoid Python’s equivalent of DLL hell). 
To install roswire from source within a virtual environment using `pipenv`:

```
$ git clone git@github.com:ChrisTimperley/rosdiscover rosdiscover
$ cd rosdiscover
$ pipenv shell
(rosdiscover) $ pip install .
(rosdiscover) $ pip install -r requirements.txt
```


### (Alternative) Docker Installation

In some cases, it may not be possible to install `rosdiscover` natively on
your machine (e.g., Mac OS or Windows machines). `rosdiscover` can be
installed on such systems by building (or downloading) and using a Docker
image for `rosdiscover`.

To build the Docker image for `rosdiscover`:

```
$ docker build -t rosdiscover .
```

Alternatively, to download a prebuilt Docker image for `rosdiscover` from DockerHub:

```
$ docker pull christimperley/rosdiscover
$ docker tag christimperley/rosdiscover rosdiscover
```

### Running `rosdiscover` via Docker

ADD INSTRUCTIONS


## Getting Started

To simulate the effects of a particular launch command, run the following:

```
$ docker run --rm -it rosdiscover
# rosdiscover launch \
    /ros_ws/src/turtlebot_simulator/turtlebot_stage/launch/turtlebot_in_stage.launch \
    --workspace /ros_ws
```

To simulate the outcome of a `rostopic` call for a particular ROS architectural
instance, given by a launch file within a workspace:

```
$ docker run --rm -it rosdiscover
# rosdiscover rostopic \
    /ros_ws/src/turtlebot_simulator/turtlebot_stage/launch/turtlebot_in_stage.launch \
    --workspace /ros_ws
```
