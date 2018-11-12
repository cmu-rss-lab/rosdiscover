from ..interpreter import model


@model('yocs_velocity_smoother', 'VelocitySmootherNodelet')
def velocity_smoother(c):
    c.read("~frequency", 20.0)
    c.read("~decel_factor", 1.0)
    c.read("~robot_feedback", 0)

    # FIXME must be present!
    # c.read("~speed_lim_v")
    # c.read("~speed_lim_w")
    # c.read("~accel_lim_v")
    # c.read("~accel_lim_w")

    c.sub("~odometry", 'nav_msgs/Odometry')
    c.sub("~robot_cmd_vel", 'geometry_msgs/Twist')
    c.sub("~raw_cmd_vel", 'geometry_msgs/Twist')

    c.pub("~smooth_cmd_vel", "geometry_msgs/Twist")

    # dynamic reconfigure
    c.pub('~parameter_descriptions', 'dynamic_reconfigure/ConfigDescription')
    c.pub('~parameter_updates', 'dynamic_reconfigure/Config')
