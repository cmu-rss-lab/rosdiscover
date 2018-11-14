from ..interpreter import model


@model('move_base', 'move_base')
def move_base(c):
    # launches an action server: move_base
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

    # action server
    c.sub("move_base/goal", "move_base_msgs/MoveBaseActionGoal")
    c.sub("move_base/cancel", "actionlib_msgs/GoalID")
    c.pub("move_base/feedback", "move_base_msgs/MoveBaseActionFeedback")
    c.pub("move_base/status", "actionlib_msgs/GoalStatusArray")
    c.pub("move_base/result", "move_base_msgs/MoveBaseActionResult")

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
        name = "global_planner"  # FIXME
        c.pub("~{}/plan".format(name), "nav_msgs/Path")
        c.read("~{}/allow_unknown".format(name), True)
        c.read("~{}/planner_window_x".format(name), 0.0)
        c.read("~{}/planner_window_y".format(name), 0.0)
        c.read("~{}/default_tolerance".format(name), 0.0)
        c.read("~{}/visualize_potential".format(name), False)

    type_global_planner = c.read("~base_global_planner", "navfn/NavfnROS")
    assert type_global_planner == 'navfn/NavfnROS'
    name_global_planner = 'NavfnROS'

    plugin_navfn()

    # load the local planner plugin
    type_local_planner = c.read("~base_local_planner",
                                "base_local_planner/TrajectoryPlannerROS")

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
