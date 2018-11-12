# rosdiscover

## Usage

Below are some instructions for running `rosdiscover` on a TurtleBot/Stage
system that was used in Phase II CP2. Support for generic ROS systems will
be added at a later point.

Firstly, we need to build a single Docker image for `rosdiscover` and the
system that we'll be analysing, as shown below.

```
docker build -t rosdiscover .
```
