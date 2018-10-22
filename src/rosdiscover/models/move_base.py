from ..vm import model


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

    c.pub("cmd_vel", "geometry_msgs/Twist")
    c.pub("~current_goal", "geometry_msgs/PoseStamped")
    c.pub("move_base/goal", "move_base_msgs/MoveBaseActionGoal")

    c.sub("move_base_simple/goal", "geometry_msgs/PoseStamped")

    c.read("~local_costmap/inscribed_radius", 0.325)
    circumscribed_radius = c.read("~local_costmap/circumscribed_radius", 0.46)
    c.read("~clearing_radius", circumscribed_radius)
    c.read("~conservative_reset_dist", 3.0)
    c.read("~shutdown_costmaps", False)
    c.read("~clearing_rotation_allowed", True)
    c.read("~recovery_behavior_enabled", True)

    # FIXME
    # LOAD: nav_core::BaseGlobalPlanner
    # load_plugin('nav_core', 'nav_core/BaseGlobalPlanner')

    # FIXME
    # controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    # load_plugin('nav_core', 'nav_core/BaseLocalPlanner')

    c.provide("make_plan", 'nav_msgs/GetPlan')
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
