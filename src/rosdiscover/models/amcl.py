# -*- coding: utf-8 -*-
# from ..interpreter import model

M_PI = 3.14159265358979323846


# Commented out to make static node recovery be used
# @model('amcl', 'amcl')
def amcl(c):
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
