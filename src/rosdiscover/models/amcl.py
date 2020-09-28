# -*- coding: utf-8 -*-
from roswire import ROSVersion

from ..interpreter import model, NodeContext

M_PI = 3.14159265358979323846


@model('amcl', 'amcl')
def amcl(c: NodeContext):
    if c.app.description.distribution.ros == ROSVersion.ROS1:
        amcl_ros1(c)
    else:
        amcl_ros2(c)


def amcl_ros2(c: NodeContext):
    # Derived from:
    #   https://github.com/ros-planning/navigation2/blob/foxy-devel/nav2_amcl/src/amcl_node.cpp

    # These are in the source but might be dummies
    c.read('~alpha1', 0.2)
    c.read('~alpha2', 0.2)
    c.read('~alpha3', 0.2)
    c.read('~alpha4', 0.2)
    c.read('~alpha5', 0.2)

    # Parameters to do with the robot frame and laser scan
    c.read('~base_frame_id', 'base_footprint')
    c.read('~beam_skip_difference', 0.5)
    c.read('~beam_skip_error_threshold', 0.9)
    c.read('~beam_skip_threshold', 0.3)
    c.read('~do_beamskip', False)

    c.read('~global_frame_id', 'map')
    c.read('~lambda_short', 0.1)

    # Laser scan parameters
    c.read('~laser_likelihood_max_dist', 2.0)
    c.read('~laser_max_range', 100.0)
    c.read('~laser_min_range', -1.0)
    c.read('~laser_model_type', 'likelihood_field')

    # Initial pose parameters, if set by the parameter file
    c.read('~set_initial_pose', False)
    c.read('~initial_pose.x', 0.0)
    c.read('~initial_pose.y', 0.0)
    c.read('~initial_pose.z', 0.0)
    c.read('~initial_pose.yaw', 0.0)

    c.read('~max_beams', 60)
    c.read('~max_particles', 2000)
    c.read('~min_particles', 500)
    c.read('~odom_frame_id', 'odom')
    c.read('~pf_err', 0.05)
    c.read('~pf_z', 0.99)

    c.read('~recovery_alpha_fast', 0.0)
    c.read('~recovery_alpha_slow', 0.0)
    c.read('~resample_interval', 1)
    c.read('~robot_model_type', 'differential')

    c.read('~save_pose_rate', 0.5)
    c.read('~sigma_hit', 0.2)
    c.read('~tf_broadcast', True)
    c.read('~transform_tolerance', 1.0)
    c.read('~update_min_a', 0.2)
    c.read('~update_min_d', 0.25)
    c.read('~z_hit', 0.5)
    c.read('~z_max', 0.05)
    c.read('~z_rand', 0.5)
    c.read('~z_short', 0.05)

    c.read('~always_reset_initial_pose', False)
    c.read('~scan_topic', 'scan')
    map_topic = c.read('~map_topic', '/map')

    c.pub('/particlecloud', 'nav2_msgs/msg/PoseArray')
    c.pub('/particle_clound', 'nav2_msgs/msg/ParticleCloud')
    c.pub('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped')

    c.sub('/clock', 'rosgraph_msgs/msg/Clock')
    c.sub('/initial_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped')
    c.sub(map_topic, 'nav_msgs/msg/OccupancyGrid')

    c.provide('/reinitialize_global_localization', 'std_srvs/srv/Empty')
    c.provide('/request_nomotion_update', 'std_srvs/srv/Empty')


def amcl_ros1(c: NodeContext):
    c.read('~use_map_topic', False)
    c.read('~first_map_only', False)
    c.read('~gui_publish_rate', -1.0)
    c.read('~save_pose_rate', 0.5)
    c.read("~gui_publish_rate", -1.0)
    c.read("~save_pose_rate", 0.5)
    c.read("~laser_min_range", -1.0)
    c.read("~laser_max_range", -1.0)
    c.read("~laser_max_beams", 30)
    c.read("~min_particles", 100)
    c.read("~max_particles", 5000)
    c.read("~kld_err", 0.01)
    c.read("~kld_z", 0.99)
    c.read("~odom_alpha1", 0.2)
    c.read("~odom_alpha2", 0.2)
    c.read("~odom_alpha3", 0.2)
    c.read("~odom_alpha4", 0.2)
    c.read("~odom_alpha5", 0.2)
    c.read("~do_beamskip", False)
    c.read("~beam_skip_distance", 0.5)
    c.read("~beam_skip_threshold", 0.3)
    c.read("~beam_skip_error_threshold_", 0.9)
    c.read("~laser_z_hit", 0.95)
    c.read("~laser_z_short", 0.1)
    c.read("~laser_z_max", 0.05)
    c.read("~laser_z_rand", 0.05)
    c.read("~laser_sigma_hit", 0.2)
    c.read("~laser_lambda_short", 0.1)
    c.read("~laser_likelihood_max_dist", 2.0)
    c.read("~laser_model_type", "likelihood_field")
    c.read("~odom_model_type", "diff")
    c.read("~update_min_d", 0.2)
    c.read("~update_min_a", M_PI / 6.0)
    c.read("~odom_frame_id", "odom")
    c.read("~base_frame_id", "base_link")
    c.read("~global_frame_id", "map")
    c.read("~resample_interval", 2)
    c.read("~transform_tolerance", 0.1)
    c.read("~recovery_alpha_slow", 0.001)
    c.read("~recovery_alpha_fast", 0.1)
    c.read("~tf_broadcast", True)
    c.read("~bag_scan_period", -1.0)

    c.pub('amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped')
    c.pub('particlecloud', 'geometry_msgs/PoseArray')

    c.sub('scan', 'sensor_msgs/LaserScan')
    c.sub('map', 'nav_msgs/OccupancyGrid')
    c.sub('initialpose', 'geometry_msgs/PoseWithCovarianceStamped')

    c.provide('global_localization', 'std_srvs/Empty')
    c.provide('request_nomotion_update', 'std_srvs/Empty')
    c.provide('set_map', 'nav_msgs/SetMap')

    # TODO add dynamic reconfigure helper
    c.pub('~parameter_descriptions', 'dynamic_reconfigure/ConfigDescription')
    c.pub('~parameter_updates', 'dynamic_reconfigure/Config')
