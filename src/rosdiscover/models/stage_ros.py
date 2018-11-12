from ..interpreter import model


@model('stage_ros', 'stageros')
def stage_ros(c):
    c.write('/use_sim_time', True)

    # FIXME implement mapName logic
    # for now, this only supports a single robot and assumes that omitRobotID
    # is true

    # n_.advertise<nav_msgs::Odometry>(mapName(ODOM, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
    c.pub('/odom', 'nav_msgs/Odometry')
    # n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
    c.pub('/base_pose_ground_truth', 'nav_msgs/Odometry')

    # new_robot->cmdvel_sub = n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1));
    c.sub('/cmd_vel', 'geometry_msgs/Twist')

    # for (size_t s = 0; s < new_robot->lasermodels.size(); ++s)
    # new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
    c.pub('/base_scan', 'sensor_msgs/LaserScan')

    # for (size_t s = 0; s < new_robot->cameramodels.size(); ++s)
    # new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
    # new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
    # new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
    c.pub('/image', 'sensor_msgs/Image')
    c.pub('/depth', 'sensor_msgs/Image')
    c.pub('/camera_info', 'sensor_msgs/CameraInfo')

    c.pub('/clock', 'rosgraph_msgs/Clock')

    c.provide('reset_positions', 'std_srvs/Empty')
