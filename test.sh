#!/bin/bash
docker run --rm -it cool \
  rosdiscover /ros_ws/src/turtlebot_simulator/turtlebot_stage/launch/turtlebot_in_stage.launch \
    --workspace /ros_ws
