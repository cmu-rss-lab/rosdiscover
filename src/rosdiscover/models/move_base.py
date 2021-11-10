from loguru import logger

from ..interpreter import model
from .plugins.navigation import NavigationPlugin


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
    c.pub("~goal", "geometry_msgs/PoseStamped")

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
        logger.debug(f"Creating costmap: {name}")
        c.sub(f"~{name}/footprint", 'geometry_msgs/Polygon')
        c.pub(f"~{name}/costmap", 'nav_msgs/OccupancyGrid')
        c.pub(f"~{name}/costmap_updates", 'nav_msgs/OccupancyGridUpdate')
        # c.pub(f"~{name}/voxel_grid", 'costmap_2d/VoxelGrid')

        c.read(f"~{name}/global_frame", "/map")
        c.read(f"~{name}/global_frame", "base_link")
        c.read(f"~{name}/transform_tolerance", 0.2)
        c.read(f"~{name}/update_frequency", 5.0)
        c.read(f"~{name}/publish_frequency", 0.0)
        c.read(f"~{name}/rolling_window", False)
        c.read(f"~{name}/always_send_full_costmap", True)

        c.read(f"~{name}/width", 10)
        c.read(f"~{name}/height", 10)
        c.read(f"~{name}/resolution", 0.05)
        c.read(f"~{name}/origin_x", 0.0)
        c.read(f"~{name}/origin_y", 0.0)

    create_costmap("global_costmap")
    create_costmap("local_costmap")

    # load the global planner plugin
    def plugin_navfn():
        name = "NavfnROS"
        c.pub(f"~{name}/plan", "nav_msgs/Path")
        c.provide(f"~{name}/make_plan", "nav_msgs/GetPlan")
        c.read(f"~{name}/allow_unknown", True)
        c.read(f"~{name}/planner_window_x", 0.0)
        c.read(f"~{name}/planner_window_y", 0.0)
        c.read(f"~{name}/default_tolerance", 0.0)
        c.read(f"~{name}/visualize_potential", False)

    type_global_planner = c.read("~base_global_planner", "navfn/NavfnROS")
    assert type_global_planner == 'navfn/NavfnROS'
    plugin_navfn()

    # load the local planner plugin
    def base_plugin_local(name):
        # type: (str) -> None
        c.pub(f"~{name}/global_plan", "nav_msgs/Path")
        c.pub(f"~{name}/local_plan", "nav_msgs/Path")
        c.sub("odom", "nav_msgs/Odometry")

    def plugin_DWAPlannerROS():  # noqa
        name = "DWAPlannerROS"
        base_plugin_local(name)

        c.read(f"~{name}/acc_lim_x", 2.5)
        c.read(f"~{name}/acc_lim_y", 2.5)
        c.read(f"~{name}/acc_lim_th", 3.2)

        c.read(f"~{name}/min_trans_vel", 0.1)
        c.read(f"~{name}/max_trans_vel", 0.55)

        c.read(f"~{name}/max_vel_x", 0.55)
        c.read(f"~{name}/min_vel_x", 0.0)
        c.read(f"~{name}/max_vel_y", 0.1)
        c.read(f"~{name}/min_vel_y", -0.1)

        c.read(f"~{name}/max_rot_vel", 1.0)
        c.read(f"~{name}/min_rot_vel", 0.4)

        c.read(f"~{name}/yaw_goal_tolerance", 0.05)
        c.read(f"~{name}/xy_goal_tolerance", 0.10)
        c.read(f"~{name}/latch_xy_goal_tolerance", False)

        c.read(f"~{name}/sim_time", 1.7)
        c.read(f"~{name}/sim_granularity", 0.025)
        c.read(f"~{name}/vx_samples", 3)
        c.read(f"~{name}/vy_samples", 10)
        c.read(f"~{name}/vth_samples", 20)
        c.read(f"~{name}/controller_frequency", 20.0)

        c.read(f"~{name}/path_distance_bias", 32.0)
        c.read(f"~{name}/goal_distance_bias", 24.0)
        c.read(f"~{name}/occdist_scale", 0.01)
        c.read(f"~{name}/forward_point_distance", 0.325)
        c.read(f"~{name}/stop_time_buffer", 0.2)
        c.read(f"~{name}/scaling_speed", 0.25)
        c.read(f"~{name}/max_scaling_factor", 0.2)

        if c.read(f"~{name}/publish_cost_grid", False) or c.read(f"~{name}/publish_cost_grid_pc", False):
            c.pub(f"~{name}/cost_cloud", "sensor_msgs/PointCloud2")

        if c.read(f"~{name}/publish_traj_pc", False):
            c.pub(f"~{name}/trajectory_cloud", "base_local_planner/MapGridCostPoint")

        c.read(f"~{name}/oscillation_reset_dist", 0.05)
        c.read(f"~{name}/prune_plan", True)

    def plugin_TrajectoryPlannerROS():  # noqa
        name = "TrajectoryPlannerROS"
        base_plugin_local(name)

        if c.read(f"~{name}/publish_cost_grid_pc", False):
            c.pub(f"~{name}/cost_cloud", "sensor_msgs/PointCloud2")

        c.read(f"~{name}/acc_lim_x", 2.5)
        c.read(f"~{name}/acc_lim_y", 2.5)
        c.read(f"~{name}/acc_lim_theta", 2.5)
        c.read(f"~{name}/max_vel_x", 2.5)
        c.read(f"~{name}/min_vel_x", 2.5)
        c.read(f"~{name}/max_vel_theta", 2.5)
        c.read(f"~{name}/min_vel_theta", 2.5)
        c.read(f"~{name}/min_in_place_cel_theta", 0.4)

        # replaces ~<name>/backup_vel since v1.3.1 of navigation stack
        c.read(f"~{name}/escape_vel", -0.1)

        if c.read(f"~{name}/holonomic_robot", True):
            c.read(f"~{name}/y_vels", [-0.3, -0.1, 0.1, 0.3])

        c.read(f"~{name}/yaw_goal_tolerance", 0.05)
        c.read(f"~{name}/xy_goal_tolerance", 0.10)
        c.read(f"~{name}/latch_xy_goal_tolerance", False)

        c.read(f"~{name}/sim_time", 1.0)
        sim_granularity = c.read(f"~{name}/sim_granularity", 0.025)
        c.read(f"~{name}/angular_sim_granularity", sim_granularity)

        c.read(f"~{name}/vx_samples", 3)
        c.read(f"~{name}/vtheta_samples", 20)

        # FIXME see http://wiki.ros.org/base_local_planner?distro=melodic
        # searches parent namespaces for controller_frequency if not present
        # in private namespace
        c.read(f"~{name}/controller_frequency", 20.0)

        c.read(f"~{name}/meter_scoring", False)
        c.read(f"~{name}/pdist_scale", 0.6)
        c.read(f"~{name}/gdist_scale", 0.8)
        c.read(f"~{name}/occdist_scale", 0.01)
        c.read(f"~{name}/heading_lookahead", 0.325)
        c.read(f"~{name}/heading_scoring", False)
        c.read(f"~{name}/heading_scoring_timestep", 0.8)
        c.read(f"~{name}/dwa", True)
        c.read(f"~{name}/global_frame_id", "odom")

        c.read("~{}/oscillation_reset_dist", 0.05)

        c.read("~{}/prune_plan", True)

    type_local_planner = c.read("~base_local_planner",
                                "base_local_planner/TrajectoryPlannerROS")
    if type_local_planner == 'base_local_planner/TrajectoryPlannerROS':
        plugin_TrajectoryPlannerROS()
    elif type_local_planner == 'dwa_local_planner/DWAPlannerROS':
        plugin_DWAPlannerROS()
    else:
        m = f"unsupported local planner: {type_local_planner}"
        raise Exception(m)

    c.provide("~make_plan", 'nav_msgs/GetPlan')
    # ! MELODIC c.provide("~clear_unknown_space", 'std_srvs/Empty')
    c.provide("~clear_costmaps", 'std_srvs/Empty')

    # move_base/src/move_base.cpp:1054
    # load_plugin('', 'clear_costmap_recovery/ClearCostmapRecovery') [conservative_reset]
    def load_recovery(name):
        # type: (str) -> None
        nh_p = f"~{name}"
        nh_blp = "~TrajectoryPlannerROS"
        c.read(f"{nh_p}/sim_granularity", 0.017)
        c.read(f"{nh_p}/frequency", 20.0)
        c.read(f"{nh_blp}/acc_lim_th", 3.2)
        c.read(f"{nh_blp}/max_rotational_vel", 1.0)
        c.read(f"{nh_blp}/min_in_place_rotational_vel", 0.4)
        c.read(f"{nh_blp}/yaw_goal_tolerance", 0.10)

    load_recovery('conservative_reset')
    load_recovery('aggressive')

    # Load navigation plugins
    global_plugins = c.read("~global_costmap/plugins")
    if global_plugins is not None:
        assert isinstance(global_plugins, list)
        for plugin_dict in global_plugins:
            assert isinstance(plugin_dict, dict)
            plugin = NavigationPlugin.from_dict(plugin_dict, c.name, "global_costmap")
            c.load_plugin(plugin)

    local_plugins = c.read("~local_costmap/plugins")
    if isinstance(local_plugins, list):
        for plugin_dict in local_plugins:
            assert isinstance(plugin_dict, dict)
            plugin = NavigationPlugin.from_dict(plugin_dict, c.name, "local_costmap")
            c.load_plugin(plugin)
