# -*- coding: utf-8 -*-
from ..interpreter import model


@model('pdw_turtlebot3_fake', 'pdw_turtlebot3_fake_node')
def turtlebot3_fake_node(c):
    c.read('tb3_model', '')
    c.read('wheel_left_joint_name', 'wheel_left_joint')
    c.read('wheel_right_joint_name', 'wheel_right_joint')
    c.read('joint_states_frame', 'base_footprint')
    c.read('odom_frame', 'odom')
    c.read('odom_frame', 'odom')
    c.read('base_frame', 'base_footprint')

    c.pub('joint_states', 'sensor_msgs/JointState')
    c.pub('odom', 'nav_msgs/Odometry')
    c.sub('cmd_vel', 'geometry_msgs/Twist')

    c.pub('/tf', 'tf2_msgs/TFMessage')
