from ..vm import model


@model('stage_ros', 'stageros')
def stage_ros(c):
    c.write('/use_sim_time', True)

    c.provide('reset_positions', 'std_srvs/Empty')

    # create pub/subs for each robot
    # - for each robot, create lasers
    # 'sensor_msgs/LaserScan'

    c.pub('/clock', 'rosgraph_msgs/Clock')
