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

### Usage

To simulate the effects of a particular launch command, run the following:

```
$ docker run --rm -it rosdiscover
# rosdiscover \
    /ros_ws/src/turtlebot_simulator/turtlebot_stage/launch/turtlebot_in_stage.launch \
    --workspace /ros_ws
```
