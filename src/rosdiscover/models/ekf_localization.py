# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext


@model('robot_localization', 'ekf_localization_node')
def ekf_localization(c: NodeContext):
    c.pub("odometry/filtered", "nav_msgs/Odometry")
    c.pub("accel/filtered", "geometry_messages/AccelWithCovarianceStamped")

    topic_index = 0
    odom_topic_name = f"~odom{topic_index}"
    while c.has_param(odom_topic_name):
        c.sub(c.read(odom_topic_name, "\\unknown"), "nav_msgs/Odometry")
        topic_index += 1
        odom_topic_name = f"~odom{topic_index}"

    topic_index = 0
    pose_topic_name = f"~pose{topic_index}"
    while c.has_param(pose_topic_name):
        c.sub(c.read(pose_topic_name, "\\unknown"), "geometry_msgs/PoseWithCovarianceStamped")
        topic_index += 1
        pose_topic_name = f"~pose{topic_index}"

    topic_index = 0
    twist_topic_name = f"~twist{topic_index}"
    while c.has_param(twist_topic_name):
        c.sub(c.read(twist_topic_name, "\\unknown"), "geometry_msgs/TwistWithCovarianceStamped")
        topic_index += 1
        twist_topic_name = f"~twist{topic_index}"

    topic_index = 0
    imu_topic_name = f"~imu{topic_index}"
    while c.has_param(imu_topic_name):
        c.sub(c.read(imu_topic_name, "\\unknown"), "sensor_msgs//Imu")
        topic_index += 1
        imu_topic_name = f"~imu{topic_index}"

    stamped_control = c.read("~stamped_control", False)
    stamped = f"geometry_msgs::Twist{'Stamped' if stamped_control else ''}"
    c.sub("set_pose", "geometry_msgs/PoseWithCovarianceStamped")
    c.sub("cmd_vel", stamped)
    c.provide("~set_pose", "geometry_msgs/PoseWithCovarianceStamped")
    c.provide("~toggle", "robot_localization/ToggleFilterProcessing")
    c.provide("~enable", "std_srvs/Empty")
