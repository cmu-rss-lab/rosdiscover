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

    # Dynamic parameters read from
    # https://github.com/ros-drivers/pointgrey_camera_driver/blob/1c71a654bea94f59396361cd735ef718f8f07011/pointgrey_camera_driver/cfg/PointGrey.cfg
    c.read("video_mode", "format7_mode0", dynamic=True)
    c.read("frame_rate", 7, dynamic=True)
    c.read("auto_exposure", True, dynamic=True)
    c.read("exposure", 1.35, dynamic=True)
    c.read("auto_shutter", True, dynamic=True)
    c.read("shutter_speed", 0.03, dynamic=True)
    c.read("auto_gain", True, dynamic=True)
    c.read("gain", 0, dynamic=True)
    c.read("pan", 0, dynamic=True)
    c.read("tilt", 0, dynamic=True)
    c.read("brightness", 0.0, dynamic=True)
    c.read("gamma", 1.0, dynamic=True)
    c.read("auto_white_balance", True, dynamic=True)
    c.read("white_balance_blue", 800, dynamic=True)
    c.read("white_balance_red", 550, dynamic=True)
    c.read("format7_roi_width", 0,)
    c.read("format7_roi_height", 0, dynamic=True)
    c.read("format7_x_offset", 0, dynamic=True)
    c.read("format7_y_offset", 0, dynamic=True)
    c.read("format7_color_coding", "raw8", dynamic=True)
    c.read("enable_trigger", False, dynamic=True)
    c.read("trigger_mode", "mode0", dynamic=True)
    c.read("trigger_source", "gpio0", dynamic=True)
    c.read("trigger_polarity", 0, dynamic=True)
    c.read("enable_trigger_delay", False, dynamic=True)
    c.read("trigger_delay", 0.0, dynamic=True)
    c.read("trigger_parameter", 0, dynamic=True)
    c.read("enable_strobe1", False, dynamic=True)
    c.read("strobe1_polarity", 0, dynamic=True)
    c.read("strobe1_delay", 0.0, dynamic=True)
    c.read("strobe1_duration", 0.0, dynamic=True)

    for image_topic in [("", "sensor_msgs/Image"), ("/compressed", "sensor_msgs/CompressedImage"),
                        ("/compressedDepth", "sensor_msgs/CompressedImage"),
                        ("/theora", "theora_image_transport/Packet")]:
        c.pub("~image_raw" + image_topic[0], image_topic[1])
    c.pub("~camera_info", "sensor_msgs/CameraInfo")
    c.pub('/diagnostics', 'diagnostics_msg/DiagnosticArray')
