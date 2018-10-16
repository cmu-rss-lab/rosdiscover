# I think we need some kind of virtual machine to emulate the behaviour of ROS
# transform workspace into a model of the system architecture
# - need to account for "actions"

# WE NEED:
# - stage_ros
# - joint_stage_publisher {BUILT-IN}: https://github.com/ros/joint_state_publisher/blob/kinetic-devel/joint_state_publisher/joint_state_publisher/joint_state_publisher
# - mobile_base_nodelet_manager
# - cmd_vel_mux (yocs_cmd_vel_mux/CmdVelMuxNodelet)
# - launch[turtlebot_navigation]/launch/includes/move_base.launch.xml
# - map_server
#   -> navigation/map_server/src/main.cpp
# launch[turtlebot_bringup]/launch/includes/robot.launch.xml


def turtlebot():
    load('stage_ros')
    # $(find turtlebot_bringup)/launch/includes/robot.launch.xml
    load('joint_state_publisher')  # https://github.com/ros/joint_state_publisher/blob/kinetic-devel/joint_state_publisher/joint_state_publisher/joint_state_publisher

    load_nodelet('mobile_base_nodelet_manager', args='manager')
    load_nodelet('cmd_vel_mux', args='load yocs_cmd_vel_mux/CmdVelMuxNodelet mobile_base_nodelet_manager')

    # $(find turtlebot_navigation)/launch/includes/move_base.launch.xml
    # - $(find turtlebot_navigation)/launch/includes/velocity_smoother.launch.xml
    # - $(find turtlebot_navigation)/launch/includes/safety_controller.launch.xml
    load('move_base')
    load_nodelet('navigation_velocity_smoother',
                 args='load yocs_velocity_smoother/VelocitySmootherNodelet mobile_base_nodelet_manager')
    load_nodelet('kobuki_safety_controller',
                 args='load kobuki_safety_controller/SafetyControllerNodelet mobile_base_nodelet_manager')

    # DONE
    load('map_server')

    # $(find turtlebot_navigation)/launch/includes/amcl/amcl.launch.xml
    load('amcl')

    load('rviz')  # ???


def move_base():
    # navigation/map_server/src/main.cpp
    name = 'move_base_node'

    # launches an action server: move_base
    read("~base_global_planner", "navfn/NavfnROS")
    read("~base_local_planner", "base_local_planner/TrajectoryPlannerROS")
    read("~global_costmap/robot_base_frame", "base_link")
    read("~global_costmap/global_frame", "/map")
    read("~planner_frequency", 0.0)
    read("~controller_frequency", 20.0)
    read("~planner_patience", 5.0)
    read("~controller_patience", 15.0)
    read("~max_planning_retries", -1)
    read("~oscillation_timeout", 0.0)
    read("~oscillation_distance", 0.5)

    pub("cmd_vel", "geometry_msgs/Twist")
    pub("~current_goal", "geometry_msgs/PoseStamped")
    pub("move_base/goal", "move_base_msgs/MoveBaseActionGoal")

    sub("move_base_simple/goal", "geometry_msgs/PoseStamped")

    read("~local_costmap/inscribed_radius", 0.325)
    read("~local_costmap/circumscribed_radius", 0.46)
    read("~clearing_radius", P["~local_costmap/circumscribed_radius"])
    read("~conservative_reset_dist", 3.0)
    read("~shutdown_costmaps", False)
    read("~clearing_rotation_allowed", True)
    read("~recovery_behavior_enabled", True)

    # LOAD: nav_core::BaseGlobalPlanner
    load_plugin('nav_core', 'nav_core/BaseGlobalPlanner')

    # controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    load_plugin('nav_core', 'nav_core/BaseLocalPlanner')

    advertise_service("make_plan", 'nav_msgs/GetPlan')  # FIXME
    advertise_service("clear_costmaps", ____)  # FIXME

    # move_base/src/move_base.cpp:1054
    # load_plugin('', 'clear_costmap_recovery/ClearCostmapRecovery') [conservative_reset]
    def load_recovery(name: str) -> None:
        nh_p = "~{}".format(name)
        nh_blp = "~TrajectoryPlannerROS"
        read("{}/sim_granularity".format(nh_p), 0.017)
        read("{}/frequency".format(nh_p), 20.0)
        read("{}/acc_lim_th".format(nh_blp), 3.2)
        read("{}/max_rotational_vel".format(nh_blp), 1.0)
        read("{}/min_in_place_rotational_vel".format(nh_blp), 0.4)
        read("{}/yaw_goal_tolerance".format(nh_blp), 0.10)

    load_recovery('conservative_reset')
    load_recovery('aggressive')


# actionlib/src/actionlib/action_client.py
def actionlib_client():
    reads += ('actionlib_client_pub_queue_size', 10)
    # TODO: rospy.remap_name(ns)
    pubs += ('___/goal', ActionGoal)  # TODO
    pubs += ('___/cancel', GoalID)
    subs += ('___/status', GoalStatusArray)
    subs += ('___/result', ActionResult)  # TODO
    subs += ('___/feedback', ActionFeedback)


# actionlib/src/actionlib/action_server.py
def actionlib_server():
    reads += ('actionlib_server_pub_queue_size', 50)
    pubs += ('___/status', GoalStatusArray)
    pubs += ('___/result', ActionResult)
    pubs += ('___/feedback', ActionFeedback)
    subs += ('___/goal', ActionGoal)
    subs += ('___/cancel', GoalID)

    # THIS IS A WEIRD ONE
    reads += ('___/status_frequency', 5.0)  # deprecated
    reads += ('actionlib_status_frequency', 5.0)

    reads += ('___/status_list_timeout', 5.0)
