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


@definition('joint_state_publisher')
def joint_state_publisher(c: NodeContext) -> None:
    # https://github.com/ros/joint_state_publisher/blob/kinetic-devel/joint_state_publisher/joint_state_publisher/joint_state_publisher

    # joint_state_publisher#L33
    def get_param(name, value=None):
        private = "~{}".format(name)
        return c.read(private, read(name, value))

    get_param("robot_description")
    get_param("dependent_joints", {})
    get_param("use_mimic_tags", True)
    get_param("use_smallest_joint_limits", True)
    get_param("zeros")

    get_param("publish_default_positions", True)
    get_param("publish_default_velocities", False)
    get_param("publish_default_efforts", False)
    get_param("use_gui", False)

    for source in get_param("source_list", []):
        c.sub(source, 'sensor_msgs/JointState')

    c.pub('joint_states', 'sensor_msgs/JointState')


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


def map_server(argv: List[str]):
    # navigation/map_server/src/main.cpp
    # anonymous: yes
    arg_map = argv[1]
    arg_resolution = argv[2]
    reads += [
        ('~frame_id', 'map'),
        ('~negate', 0),
        ('~occupied_thresh', 0.65),
        ('~free_thresh', 0.196)
    ]
    services += [
        ('static_map', 'nav_msgs/GetMap')
    ]
    pubs += [
        ('map_metadata', 'nav_msgs/MapMetaData'),
        ('map', 'nav_msgs/OccupancyGrid')
    ]


def amcl():
    # navigation/amcl/src/amcl_node.cpp
    reads += [
        ('~use_map_topic', False),
        ('~first_map_only', False),
        ('~gui_publish_rate', -1.0),
        ('~save_pose_rate', 0.5),
        ("~gui_publish_rate", -1.0),
        ("~save_pose_rate", 0.5),
        ("~laser_min_range", -1.0),
        ("~laser_max_range", -1.0),
        ("~laser_max_beams", 30),
        ("~min_particles", 100),
        ("~max_particles", 5000),
        ("~kld_err", 0.01),
        ("~kld_z", 0.99),
        ("~odom_alpha1", 0.2),
        ("~odom_alpha2", 0.2),
        ("~odom_alpha3", 0.2),
        ("~odom_alpha4", 0.2),
        ("~odom_alpha5", 0.2),
        ("~do_beamskip", False),
        ("~beam_skip_distance", 0.5),
        ("~beam_skip_threshold", 0.3),
        ("~beam_skip_error_threshold_", 0.9),
        ("~laser_z_hit", 0.95),
        ("~laser_z_short", 0.1),
        ("~laser_z_max", 0.05),
        ("~laser_z_rand", 0.05),
        ("~laser_sigma_hit", 0.2),
        ("~laser_lambda_short", 0.1),
        ("~laser_likelihood_max_dist", 2.0),
        ("~laser_model_type", "likelihood_field"),
        ("~odom_model_type", "diff"),
        ("~update_min_d", 0.2),
        ("~update_min_a", M_PI/6.0),  # FIXME
        ("~odom_frame_id", "odom"),
        ("~base_frame_id", "base_link"),
        ("~global_frame_id", "map"),
        ("~resample_interval", 2),
        ("~transform_tolerance", 0.1),
        ("~recovery_alpha_slow", 0.001),
        ("~recovery_alpha_fast", 0.1),
        ("~tf_broadcast", True),
        ("~bag_scan_period", -1.0)
    ]
    pubs += [
        ('amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'),
        ('particlecloud', 'geometry_msgs/PoseArray')
    ]
    subs += [
        ('scan', 'sensor_msgs/LaserScan')
        ('map', 'nav_msgs::OccupancyGrid'),
        ('initialpose', 'geometry_msgs/PoseWithCovarianceStamped'),
    ]
    services += [
        ('global_localization', 'std_srvs/Empty'),
        ('request_nomotion_update', 'std_srvs/Empty'),
        ('set_map', 'nav_msgs/SetMap')
    ]


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
