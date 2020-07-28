from ..interpreter import model


@model('fetch_teleop', 'fetch_telop')
def fetch_teleop(c):
    ...
    # Parameter usage
    c.read("use_max", True)

    # Topic subscriptions
    c.sub("/odom", "nav_msgs/Odometry")
    c.pub("/teleop/cmd_vel", "geometry_msgs/Twist")

    # Services called/provided
    c.call("/cmd_vel_mux", "topic_tools/MuxSelect")
