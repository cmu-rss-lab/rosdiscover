# -*- coding: utf-8 -*-
from roswire import ROSDistribution

from ..interpreter import model, NodeContext


@model('autorally_control', 'lap_stats')
def lap_stats(c: NodeContext) -> None:
    # For indigo and kinetic
    prefix = "/stat_tracker/controller_type"
    c.read(f"{prefix}/hz")
    c.read(f"{prefix}/num_timesteps")
    c.read(f"{prefix}/tag")
    c.read(f"{prefix}/gamma")
    c.read(f"{prefix}/num_iters")
    c.read(f"{prefix}/init_steering")
    c.read(f"{prefix}/init_throttle")
    c.read(f"{prefix}/steering_var")
    c.read(f"{prefix}/throttle_var")
    c.read(f"{prefix}/max_throttle")
    c.read(f"{prefix}/desired_speed")
    c.read(f"{prefix}/speed_coefficient")
    c.read(f"{prefix}/track_coefficient")
    c.read(f"{prefix}/max_slip_angle")
    c.read(f"{prefix}/slip_penalty")
    c.read(f"{prefix}/track_slop")
    c.read(f"{prefix}/crash_coeff")
    c.read(f"{prefix}/map_path")

    c.pub('lap_stats', 'autorally_msgs/pathIntegralStats')
    if c.ros_distro < ROSDistribution.MELODIC:
        c.sub('/pose_estimate', 'nav_msgs/Odometry')
    else:
        c.sub('/mppi_controller/subscribedPose', 'nav_msgs/Odometry')
