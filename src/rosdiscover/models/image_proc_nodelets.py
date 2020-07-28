# -*- coding: utf-8 -*-
from ..interpreter import model

IMAGE_TOPIC_TYPE = 'sensor_msgs/Image'
IMAGE_PROC_PKG = 'image_proc'


@model(IMAGE_PROC_PKG, "image_proc")
def image_proc(c):
    c.sub('image_raw', IMAGE_TOPIC_TYPE)
    c.sub('camera_info', 'sensor_msgs/CameraInfo')

    c.pub('image_mono', IMAGE_TOPIC_TYPE)
    c.pub('image_rect', IMAGE_TOPIC_TYPE)
    c.pub('image_color', IMAGE_TOPIC_TYPE)
    c.pub('image_rect_color', IMAGE_TOPIC_TYPE)

    c.read('~queue_size', 5)


@model(IMAGE_PROC_PKG, "debayer")
def debayer(c):
    c.mark_nodelet()
    c.pub('image_mono', IMAGE_TOPIC_TYPE)
    c.pub('image_color', IMAGE_TOPIC_TYPE)

    c.sub('image_raw', IMAGE_TOPIC_TYPE)

    c.read('~debayer', 0)


@model(IMAGE_PROC_PKG, 'rectify')
def rectify(c):
    c.mark_nodelet()

    c.sub('image_mono', IMAGE_TOPIC_TYPE)
    c.sub('camera_info', 'sensor_msgs/CameraInfo')

    c.pub('image_rect', IMAGE_TOPIC_TYPE)

    c.read('~queue_size', 5)
    c.read('~interpolation', 1)


@model(IMAGE_PROC_PKG, 'crop_decimate')
def crop_decimate(c):
    c.mark_nodelet()

    c.sub('camera/image_raw', IMAGE_TOPIC_TYPE)
    c.sub('camera/camera_info', 'sensor_msgs/CameraInfo')

    c.pub('camera_out/image_raw', IMAGE_TOPIC_TYPE)
    c.pub('camera_out/camera_info', 'sensor_msgs/CameraInfo')

    c.read('~queue_size', 5)
    c.read('~decimation_x', 1, True)
    c.read('~decimation_y', 1, True)
    c.read('~x_offset', 0, True)
    c.read('~y_offset', 0, True)
    c.read('~width', 0, True)
    c.read('~height', 0, True)
    c.read('~interpolation', 0, True)


@model(IMAGE_PROC_PKG, 'resize')
def resize(c):
    c.mark_nodelet()

    c.sub('image', IMAGE_TOPIC_TYPE)
    c.sub('camera_info', 'sensor_msgs/CameraInfo')

    c.pub('~image', IMAGE_TOPIC_TYPE)
    c.pub('~camera_info', 'sensor_msgs/CameraInfo')
