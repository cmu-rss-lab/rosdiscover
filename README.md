# rosdiscover

## Getting Started

Below are some instructions for running `rosdiscover` on a TurtleBot/Stage
system that was used in Phase II CP2. Support for generic ROS systems will
be added at a later point.


### Installation

Build a single Docker image for `rosdiscover` and the system under analysis
using the provided Dockerfile, as shown below.

```
$ docker build -t rosdiscover .
```

Install dependencies:

```
(venv) $ pip install .
(venv) $ pip install -r requirements.txt
```

### Usage

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


### Example Ground Truth

```
$ xhost local:root
$ docker run --rm \
  -e DISPLAY=unix${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -it discover
~ roslaunch /ros_ws/src/turtlebot_simulator/turtlebot_stage/launch/turtlebot_in_stage.launch
```
