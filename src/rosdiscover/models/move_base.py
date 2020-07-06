from ..interpreter import model


@model('move_base', 'move_base')
def move_base(c):
    c.read("~base_global_planner", "navfn/NavfnROS")
    c.read("~base_local_planner", "base_local_planner/TrajectoryPlannerROS")
    c.read("~global_costmap/robot_base_frame", "base_link")
    c.read("~global_costmap/global_frame", "/map")
    c.read("~planner_frequency", 0.0)
    c.read("~controller_frequency", 20.0)
    c.read("~planner_patience", 5.0)
    c.read("~controller_patience", 15.0)
    c.read("~max_planning_retries", -1)
    c.read("~oscillation_timeout", 0.0)
    c.read("~oscillation_distance", 0.5)

    c.action_server("move_base", "move_base_msgs/MoveBaseAction")

    c.sub("move_base_simple/goal", "geometry_msgs/PoseStamped")

    c.pub("cmd_vel", "geometry_msgs/Twist")
    c.pub("~current_goal", "geometry_msgs/PoseStamped")

    c.read("~local_costmap/inscribed_radius", 0.325)
    circumscribed_radius = c.read("~local_costmap/circumscribed_radius", 0.46)
    c.read("~clearing_radius", circumscribed_radius)
    c.read("~conservative_reset_dist", 3.0)
    c.read("~shutdown_costmaps", False)
    c.read("~clearing_rotation_allowed", True)
    c.read("~recovery_behavior_enabled", True)

    # FIXME load costmap plugins
    # SEE http://wiki.ros.org/costmap_2d?distro=melodic
    def create_costmap(name):
        c.sub("~{}/footprint".format(name), 'geometry_msgs/Polygon')
        c.pub("~{}/costmap".format(name), 'nav_msgs/OccupancyGrid')
        c.pub("~{}/costmap_updates".format(name), 'nav_msgs/OccupancyGridUpdate')
        c.pub("~{}/voxel_grid".format(name), 'costmap_2d/VoxelGrid')

        c.read("~{}/global_frame".format(name), "/map")
        c.read("~{}/global_frame".format(name), "base_link")
        c.read("~{}/transform_tolerance".format(name), 0.2)
        c.read("~{}/update_frequency".format(name), 5.0)
        c.read("~{}/publish_frequency".format(name), 0.0)
        c.read("~{}/rolling_window".format(name), False)
        c.read("~{}/always_send_full_costmap".format(name), True)

        c.read("~{}/width".format(name), 10)
        c.read("~{}/height".format(name), 10)
        c.read("~{}/resolution".format(name), 0.05)
        c.read("~{}/origin_x".format(name), 0.0)
        c.read("~{}/origin_y".format(name), 0.0)

    create_costmap("global_costmap")
    create_costmap("local_costmap")

    # load the global planner plugin
    def plugin_navfn():
        name = "NavfnROS"
        c.pub("~{}/plan".format(name), "nav_msgs/Path")
        c.read("~{}/allow_unknown".format(name), True)
        c.read("~{}/planner_window_x".format(name), 0.0)
        c.read("~{}/planner_window_y".format(name), 0.0)
        c.read("~{}/default_tolerance".format(name), 0.0)
        c.read("~{}/visualize_potential".format(name), False)

    type_global_planner = c.read("~base_global_planner", "navfn/NavfnROS")
    assert type_global_planner == 'navfn/NavfnROS'
    plugin_navfn()

    # load the local planner plugin
    def base_plugin_local(name):
        # type: (str) -> None
        c.pub("~{}/global_plan".format(name), "nav_msgs/Path")
        c.pub("~{}/local_plan".format(name), "nav_msgs/Path")
        c.sub("odom", "sensor_msgs/PointCloud2")

    def plugin_DWAPlannerROS():  # noqa
        name = "DWAPlannerROS"
        base_plugin_local(name)

        c.read("~{}/acc_lim_x".format(name), 2.5)
        c.read("~{}/acc_lim_y".format(name), 2.5)
        c.read("~{}/acc_lim_th".format(name), 3.2)

        c.read("~{}/min_trans_vel".format(name), 0.1)
        c.read("~{}/max_trans_vel".format(name), 0.55)

        c.read("~{}/max_vel_x".format(name), 0.55)
        c.read("~{}/min_vel_x".format(name), 0.0)
        c.read("~{}/max_vel_y".format(name), 0.1)
        c.read("~{}/min_vel_y".format(name), -0.1)

        c.read("~{}/max_rot_vel".format(name), 1.0)
        c.read("~{}/min_rot_vel".format(name), 0.4)

        c.read("~{}/yaw_goal_tolerance".format(name), 0.05)
        c.read("~{}/xy_goal_tolerance".format(name), 0.10)
        c.read("~{}/latch_xy_goal_tolerance".format(name), False)

        c.read("~{}/sim_time".format(name), 1.7)
        c.read("~{}/sim_granularity".format(name), 0.025)
        c.read("~{}/vx_samples".format(name), 3)
        c.read("~{}/vy_samples".format(name), 10)
        c.read("~{}/vth_samples".format(name), 20)
        c.read("~{}/controller_frequency".format(name), 20.0)

        c.read("~{}/path_distance_bias".format(name), 32.0)
        c.read("~{}/goal_distance_bias".format(name), 24.0)
        c.read("~{}/occdist_scale".format(name), 0.01)
        c.read("~{}/forward_point_distance".format(name), 0.325)
        c.read("~{}/stop_time_buffer".format(name), 0.2)
        c.read("~{}/scaling_speed".format(name), 0.25)
        c.read("~{}/max_scaling_factor".format(name), 0.2)

        if c.read("~{}/publish_cost_grid".format(name), False):
            c.pub("~{}/cost_cloud".format(name), "sensor_msgs/PointCloud2")

        c.read("~{}/oscillation_reset_dist".format(name), 0.05)
        c.read("~{}/prune_plan".format(name), True)

    def plugin_TrajectoryPlannerROS():  # noqa
        name = "TrajectoryPlannerROS"
        base_plugin_local(name)

        if c.read("~{}/publish_cost_grid_pc".format(name), False):
            c.pub("~{}/cost_cloud".format(name), "sensor_msgs/PointCloud2")

        c.read("~{}/acc_lim_x".format(name), 2.5)
        c.read("~{}/acc_lim_y".format(name), 2.5)
        c.read("~{}/acc_lim_theta".format(name), 2.5)
        c.read("~{}/max_vel_x".format(name), 2.5)
        c.read("~{}/min_vel_x".format(name), 2.5)
        c.read("~{}/max_vel_theta".format(name), 2.5)
        c.read("~{}/min_vel_theta".format(name), 2.5)
        c.read("~{}/min_in_place_cel_theta".format(name), 0.4)

        # replaces ~<name>/backup_vel since v1.3.1 of navigation stack
        c.read("~{}/escape_vel".format(name), -0.1)

        if c.read("~{}/holonomic_robot".format(name), True):
            c.read("~{}/y_vels".format(name), [-0.3, -0.1, 0.1, 0.3])

        c.read("~{}/yaw_goal_tolerance".format(name), 0.05)
        c.read("~{}/xy_goal_tolerance".format(name), 0.10)
        c.read("~{}/latch_xy_goal_tolerance".format(name), False)

        c.read("~{}/sim_time".format(name), 1.0)
        sim_granularity = c.read("~{}/sim_granularity".format(name), 0.025)
        c.read("~{}/angular_sim_granularity".format(name), sim_granularity)

        c.read("~{}/vx_samples".format(name), 3)
        c.read("~{}/vtheta_samples".format(name), 20)

        # FIXME see http://wiki.ros.org/base_local_planner?distro=melodic
        # searches parent namespaces for controller_frequency if not present
        # in private namespace
        c.read("~{}/controller_frequency".format(name), 20.0)

        c.read("~{}/meter_scoring".format(name), False)
        c.read("~{}/pdist_scale".format(name), 0.6)
        c.read("~{}/gdist_scale".format(name), 0.8)
        c.read("~{}/occdist_scale".format(name), 0.01)
        c.read("~{}/heading_lookahead".format(name), 0.325)
        c.read("~{}/heading_scoring".format(name), False)
        c.read("~{}/heading_scoring_timestep".format(name), 0.8)
        c.read("~{}/dwa".format(name), True)
        c.read("~{}/global_frame_id".format(name), "odom")

        c.read("~{}/oscillation_reset_dist", 0.05)

        c.read("~{}/prune_plan", True)

    type_local_planner = c.read("~base_local_planner",
                                "base_local_planner/TrajectoryPlannerROS")
    if type_local_planner == 'base_local_planner/TrajectoryPlannerROS':
        plugin_TrajectoryPlannerROS()
    elif type_local_planner == 'dwa_local_planner/DWAPlannerROS':
        plugin_DWAPlannerROS()
    else:
        m = "unsupported local planner: {}".format(type_local_planner)
        raise Exception(m)

    c.provide("make_plan", 'nav_msgs/GetPlan')
    c.provide("clear_unknown_space", 'std_srvs/Empty')
    c.provide("clear_costmaps", 'std_srvs/Empty')

    # move_base/src/move_base.cpp:1054
    # load_plugin('', 'clear_costmap_recovery/ClearCostmapRecovery') [conservative_reset]
    def load_recovery(name):
        # type: (str) -> None
        nh_p = "~{}".format(name)
        nh_blp = "~TrajectoryPlannerROS"
        c.read("{}/sim_granularity".format(nh_p), 0.017)
        c.read("{}/frequency".format(nh_p), 20.0)
        c.read("{}/acc_lim_th".format(nh_blp), 3.2)
        c.read("{}/max_rotational_vel".format(nh_blp), 1.0)
        c.read("{}/min_in_place_rotational_vel".format(nh_blp), 0.4)
        c.read("{}/yaw_goal_tolerance".format(nh_blp), 0.10)

    load_recovery('conservative_reset')
    load_recovery('aggressive')
