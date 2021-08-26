# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext


@model("pointgrey_camera_driver", "PointGreyCameraNodelet")
def pointgrey_camera_driver(c: NodeContext) -> None:
    c.mark_nodelet()

    # How to derive dynamic parameters from:
    # https://github.com/ros-drivers/pointgrey_camera_driver/blob/1c71a654bea94f59396361cd735ef718f8f07011/pointgrey_camera_driver/src/nodelet.cpp#L270
    c.read('serial', 0)
    c.read('packet_size', 1400)
    c.read('auto_packet_size', True)
    c.read('packet_delay', 4000)
    c.read('camera_info_url', '')
    c.read('frame_id', 'camera')

    c.read('desired_freq', 7.0)
    c.read('min_freq', 7.0)
    c.read('max_freq', 7.0)
    c.read('freq_tolerance', 0.1)
    c.read('window_size', 100)
    c.read('min_acceptable_delay', 0.0)
    c.read('max_acceptable_delay', 0.2)

    c.pub('image_raw', 'sensor_msgs/Image')
    c.pub('image', 'wfov_camera_msgs/WFOVImage')
