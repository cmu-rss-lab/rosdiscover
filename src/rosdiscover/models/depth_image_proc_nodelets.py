from ..interpreter import model

POINTCLOUD2 = 'sensor_msgs/PointCloud2'

MSGS_CAMERA_INFO = 'sensor_msgs/CameraInfo'

IMAGE_TOPIC_TYPE = 'sensor_msgs/Image'

DEPTH_IMAGE_PROC_PKG = 'depth_image_proc'

@model(DEPTH_IMAGE_PROC_PKG, 'convert_metric')
def convert_metric(c):
    c.markAsNodelet()

    c.sub ('image_raw', IMAGE_TOPIC_TYPE)
    c.pub ('image', IMAGE_TOPIC_TYPE)

@model(DEPTH_IMAGE_PROC_PKG, 'disparity')
def disparity(c):
    c.markAsNodelet()

    c.sub('left/image_rect', IMAGE_TOPIC_TYPE)
    c.sub('right/camera_info', MSGS_CAMERA_INFO)

    c.read('min_range', 0.0)
    c.read('max_range', 0.0)
    c.read('delta_d', 0.125)
    c.read('queue_size', 5)

@model(DEPTH_IMAGE_PROC_PKG, 'point_cloud_xyz')
def point_cloud_xyz(c):
    c.markAsNodelet()

    c.sub('camera_info', MSGS_CAMERA_INFO)
    c.sub('image_rect', IMAGE_TOPIC_TYPE)

    c.pub('points', POINTCLOUD2)

    c.read('queue_size', 5)

@model(DEPTH_IMAGE_PROC_PKG, 'point_cloud_xyzrgb')
def point_cloud_xyzrgb(c):
    c.markAsNodelet()

    c.sub('rgb/camera_info', MSGS_CAMERA_INFO)
    c.sub('rgb/image_rect_color', IMAGE_TOPIC_TYPE)
    c.sub('depth_registered/image_rect', IMAGE_TOPIC_TYPE)

    c.pub('depth_registered/points', POINTCLOUD2)

    c.read('queue_size', 5)

@model(DEPTH_IMAGE_PROC_PKG, 'register')
def register(c):
    c.markAsNodelet()

    c.sub('rgb/camera_info', MSGS_CAMERA_INFO)
    c.sub('depth/camera_info', MSGS_CAMERA_INFO)
    c.sub('depth/image_rect', IMAGE_TOPIC_TYPE)

    c.pub('depth_registered/camera_info', MSGS_CAMERA_INFO)
    c.pub('depth_registered/image_rect', IMAGE_TOPIC_TYPE)

    c.read('queue_size', 5)

    #c.tf('depth_optical_frame', '/rgb_optical_frame')
