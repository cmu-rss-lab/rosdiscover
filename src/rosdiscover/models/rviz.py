from ..interpreter import model


@model('rviz', 'rviz')
def rviz(c):
    c.provide('reload_shaders', 'std_srvs/Empty')

    # https://github.com/ros-visualization/rviz/blob/070835c426b8982e304b38eb4a9c6eb221155d5f/src/rviz/default_plugin/marker_array_display.cpp
    # FIXME
    c.sub('visualization_marker_array', 'visualization_msgs/MarkerArray')
