# -*- coding: utf-8 -*-
from ..interpreter import model


@model('stage_ros', 'stageros')
def stage_ros(c):
    c.write('/use_sim_time', True)

    # FIXME implement mapName logic
    # for now, this only supports a single robot and assumes that omitRobotID
    # is true

    c.pub('/odom', 'nav_msgs/Odometry')
    c.pub('/base_pose_ground_truth', 'nav_msgs/Odometry')
    c.pub('/base_scan', 'sensor_msgs/LaserScan')
    c.pub('/image', 'sensor_msgs/Image')
    c.pub('/depth', 'sensor_msgs/Image')
    c.pub('/camera_info', 'sensor_msgs/CameraInfo')
    c.pub('/clock', 'rosgraph_msgs/Clock')

    c.sub('/cmd_vel', 'geometry_msgs/Twist')

    c.provide('reset_positions', 'std_srvs/Empty')
