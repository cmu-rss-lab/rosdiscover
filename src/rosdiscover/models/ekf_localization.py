# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext

@model('robot_localization', 'ekf_localization_node')
def ekf_localization(c: NodeContext):
    c.pub("odometry/filtered", "nav_msgs/Odometry")
    c.pub("accel/filtered", "geometry_messages/AccelWithCovarianceStamped")

    topicInd = 0
    odomTopicName = f"~odom{topicInd}"
    while c.has_param(odomTopicName):
        c.sub(c.read(odomTopicName, "\\unknown"), "nav_msgs/Odometry")
        topicInd += 1
        odomTopicName = f"~odom{topicInd}"

    topicInd = 0
    poseTopicName = f"~pose{topicInd}"
    while c.has_param(poseTopicName):
        c.sub(c.read(poseTopicName, "\\unknown"), "geometry_msgs/PoseWithCovarianceStamped")
        topicInd += 1
        poseTopicName = f"~pose{topicInd}"


    topicInd = 0
    twistTopicName = f"~twist{topicInd}"
    while c.has_param(twistTopicName):
        c.sub(c.read(twistTopicName, "\\unknown"), "geometry_msgs/TwistWithCovarianceStamped")
        topicInd += 1
        twistTopicName = f"~twist{topicInd}"

    topicInd = 0
    imuTopicName = f"~imu{topicInd}"
    while c.has_param(imuTopicName):
        c.sub(c.read(imuTopicName, "\\unknown"), "sensor_msgs//Imu")
        topicInd += 1
        imuTopicName = f"~imu{topicInd}"


    stamped_control = c.read("~stamped_control", False)
    stamped = f"geometry_msgs::Twist{'Stamped' if stamped_control else ''}"
    c.sub("set_pose", "geometry_msgs/PoseWithCovarianceStamped")
    c.sub("cmd_vel", stamped)
    c.provide("~set_pose", "geometry_msgs/PoseWithCovarianceStamped")
    c.provide("~toggle", "robot_localization/ToggleFilterProcessing")
    c.provide("~enable", "std_srvs/Empty")

