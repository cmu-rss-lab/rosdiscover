# rosdiscover


## Installation

See below for two different methods of installing rosdiscover. Both methods
require that Docker is installed on your machine and that your user belongs
to the `docker` group (i.e., `sudo` isn't required to run `docker` commands).


### Native Installation

To avoid interfering with the rest of your system (i.e., to avoid Python’s
equivalent of DLL hell), we strongly recommend that ROSWire is installed
within a virtualenv or pipenv (pipenv is preferred). To install roswire
from source within a virtual environment:

```
$ git clone git@github.com:ChrisTimperley/rosdiscover rosdiscover
$ cd rosdiscover
$ pipenv shell
(rosdiscover) $ pip install .
```

Note that installing rosdiscover to your host machine using the method
described above requires that Python 3.6+ is installed on your machine.

### (Alternative) Docker Installation


Build a single Docker image for `rosdiscover` and the system under analysis
using the provided Dockerfile, as shown below.

```
$ docker build -t rosdiscover .
```

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
